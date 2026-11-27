// main.cpp — Matter Plant Watering System
//
// Architecture: non-blocking FSM, Matter-over-WiFi (ESP32/C6) or Thread (H2).
// No dependency on the high-level Matter.h wrapper; uses MatterInit + direct
// esp_matter SDK calls throughout.
//
// Hardware — GPIO pins configurable via platformio.ini build flags:
//   VALVE_PIN_0 … VALVE_PIN_9  — solenoid valve relay/MOSFET outputs
//   MOTOR_PIN                  — pump motor relay/MOSFET output
//   STATUS_LED_PIN             — status LED (built-in or external)
//   ACTION_BUTTON_PIN          — decommission button (hold 5 s to factory-reset)
//
// Matter device type: 10× on_off_plugin_unit (OnOff cluster)
//   Each switch in the Matter controller maps to one solenoid valve.
//   Motor turns ON when any valve is open; OFF when all valves are closed.

#include <Arduino.h>

#if CONFIG_ENABLE_MATTER_OVER_THREAD
#include <esp_openthread.h>
#include <openthread/thread.h>
#else
#include <WiFi.h>
#include <esp_wifi.h>
#endif

#include "MatterInit.h"
#include "MatterDeviceProvider.h"
#include "MatterValve.h"
#include "ValveControl.h"

// ─── Build-flag defaults ─────────────────────────────────────────────────────
#ifndef ACTION_BUTTON_PIN
#define ACTION_BUTTON_PIN 9
#endif
#ifndef STATUS_LED_PIN
#define STATUS_LED_PIN 2
#endif
#ifndef MOTOR_PIN
#define MOTOR_PIN 26
#endif

// How long ACTION_BUTTON must be held continuously to trigger decommission (ms).
static constexpr uint32_t DECOMMISSION_HOLD_MS = 5000;

// ─── Matter endpoints & valve control ────────────────────────────────────────
static MatterValve  gValves[NUM_VALVES];
static ValveControl gValveCtrl;

// ─── FSM ─────────────────────────────────────────────────────────────────────
enum class State : uint8_t {
    BOOT,              // GPIO init, serial, optional WiFi connect
    MATTER_INIT,       // Create Matter node + endpoints, start stack
    NOT_COMMISSIONED,  // Waiting for BLE/IP commissioning (LED slow-blink)
    CONNECTING,        // Commissioned; waiting for WiFi/Thread network (LED fast-blink)
    OPERATIONAL,       // Fully connected; Matter commands active (LED solid)
    DECOMMISSIONING,   // Factory reset in progress
};
static State gState = State::BOOT;

// ─── Decommission button ─────────────────────────────────────────────────────
static uint32_t gButtonHoldStart = 0;   // millis() when button went LOW; 0 = not held

// ─── LED blink timing ────────────────────────────────────────────────────────
static uint32_t gLedLastToggle = 0;
static bool     gLedOn         = false;

// ─── WiFi (ESP32 / ESP32-C6 without BLE commissioning) ───────────────────────
#if !CONFIG_ENABLE_MATTER_OVER_THREAD && defined(WIFI_SSID)
static bool gWifiStarted = false;
#endif

// ─── Power-saving flag ───────────────────────────────────────────────────────
static bool gModemSleepEnabled = false;

// ════════════════════════════════════════════════════════════════════════════
// ── LED helper ───────────────────────────────────────────────────────────────
// ════════════════════════════════════════════════════════════════════════════

static void updateLED(State state) {
    uint32_t now = millis();
    switch (state) {
        case State::NOT_COMMISSIONED: {
            // Slow blink: 1 Hz (500 ms on, 500 ms off)
            if (now - gLedLastToggle >= 500) {
                gLedLastToggle = now;
                gLedOn = !gLedOn;
                digitalWrite(STATUS_LED_PIN, gLedOn ? HIGH : LOW);
            }
            break;
        }
        case State::CONNECTING: {
            // Fast blink: 4 Hz (125 ms on, 125 ms off)
            if (now - gLedLastToggle >= 125) {
                gLedLastToggle = now;
                gLedOn = !gLedOn;
                digitalWrite(STATUS_LED_PIN, gLedOn ? HIGH : LOW);
            }
            break;
        }
        case State::OPERATIONAL: {
            // Solid ON
            if (!gLedOn) {
                gLedOn = true;
                digitalWrite(STATUS_LED_PIN, HIGH);
            }
            break;
        }
        default: {
            // All other states: LED off
            if (gLedOn) {
                gLedOn = false;
                digitalWrite(STATUS_LED_PIN, LOW);
            }
            break;
        }
    }
}

// ════════════════════════════════════════════════════════════════════════════
// ── Button handler ───────────────────────────────────────────────────────────
// ════════════════════════════════════════════════════════════════════════════

// Non-blocking: must be called every loop() iteration.
// Transitions to DECOMMISSIONING if button is held for DECOMMISSION_HOLD_MS.
static void handleButton() {
    if (gState == State::DECOMMISSIONING) return;

    bool held = (digitalRead(ACTION_BUTTON_PIN) == LOW);
    if (held) {
        if (gButtonHoldStart == 0) gButtonHoldStart = millis();
        if (millis() - gButtonHoldStart >= DECOMMISSION_HOLD_MS) {
            gState = State::DECOMMISSIONING;
        }
    } else {
        gButtonHoldStart = 0;
    }
}

// ════════════════════════════════════════════════════════════════════════════
// ── Arduino entry points ─────────────────────────────────────────────────────
// ════════════════════════════════════════════════════════════════════════════

void setup() {
    Serial.begin(115200);
    Serial.println("\n=== Matter Plant Watering System ===");

    // All valve/motor GPIOs OUTPUT + OFF immediately — avoids relay chatter
    // during the Matter stack initialization phase.
    gValveCtrl.begin();

    // Status LED and action button
    pinMode(STATUS_LED_PIN, OUTPUT);
    digitalWrite(STATUS_LED_PIN, LOW);
    pinMode(ACTION_BUTTON_PIN, INPUT_PULLUP);

    gState = State::BOOT;
}

void loop() {
    handleButton();

    switch (gState) {

    // ── BOOT ─────────────────────────────────────────────────────────────────
    // For WiFi-only ESP32 (no BLE commissioning): connect to WiFi first, then
    // proceed to MATTER_INIT. For C6/H2 (BLE commissioning): skip straight through.
    case State::BOOT: {
#if !CONFIG_ENABLE_MATTER_OVER_THREAD && defined(WIFI_SSID)
        // ESP32: WiFi credentials are baked in via build flags from secrets.ini.
        // Stay in BOOT until connected — non-blocking polling.
        if (!gWifiStarted) {
            Serial.printf("Connecting to WiFi: %s\n", WIFI_SSID);
            WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
            gWifiStarted = true;
        }
        if (WiFi.status() != WL_CONNECTED) {
            static uint32_t lastPrint = 0;
            if (millis() - lastPrint >= 3000) {
                lastPrint = millis();
                Serial.print(".");
            }
            break;  // stay in BOOT
        }
        Serial.printf("\nWiFi connected: %s\n", WiFi.localIP().toString().c_str());
#endif
        gState = State::MATTER_INIT;
        break;
    }

    // ── MATTER_INIT ──────────────────────────────────────────────────────────
    // Create the Matter node, register all 10 valve endpoints, start the stack.
    // This is done once per power-on (no deep sleep, so setup() runs once).
    case State::MATTER_INIT: {
        if (!MatterInit::init()) {
            Serial.println("ERROR: Matter node init failed — halting");
            while (true) delay(1000);
        }

        // Register 10 valve endpoints (order determines endpoint IDs: EP1–EP10).
        // Wire each valve's onChange callback to ValveControl.
        for (uint8_t i = 0; i < NUM_VALVES; i++) {
            gValves[i].onChange = [i](uint8_t /*idx*/, bool on) {
                gValveCtrl.setValve(i, on);
            };
            if (!gValves[i].begin(i, /*initialState=*/false)) {
                Serial.printf("ERROR: failed to create valve endpoint %u\n", i);
                while (true) delay(1000);
            }
        }

        if (!MatterInit::start()) {
            Serial.println("ERROR: Matter stack start failed — halting");
            while (true) delay(1000);
        }

        MatterDeviceProvider::printPairingInfo();

        gState = MatterInit::isCommissioned() ? State::CONNECTING : State::NOT_COMMISSIONED;
        break;
    }

    // ── NOT_COMMISSIONED ─────────────────────────────────────────────────────
    // Waiting for Matter commissioning via BLE (C6/H2) or mDNS/IP (ESP32).
    case State::NOT_COMMISSIONED: {
        if (MatterInit::isCommissioned()) {
            Serial.println("Commissioning complete — waiting for network connection");
            gState = State::CONNECTING;
            break;
        }
        static uint32_t lastPrint = 0;
        if (millis() - lastPrint >= 10000) {
            lastPrint = millis();
            Serial.printf("Waiting for Matter commissioning… manual pairing code: %s\n",
                          MatterDeviceProvider::getManualPairingCode().c_str());
        }
        break;
    }

    // ── CONNECTING ───────────────────────────────────────────────────────────
    // Commissioned; waiting for WiFi station or Thread attachment.
    case State::CONNECTING: {
        if (!MatterInit::isCommissioned()) {
            // Fabric was removed while connecting — go back to commissioning.
            Serial.println("Fabric removed — back to commissioning");
            gState = State::NOT_COMMISSIONED;
            break;
        }
        if (MatterInit::isConnected()) {
            Serial.println("Network connected — entering OPERATIONAL");

            // Enable WiFi modem sleep once connected to reduce idle current.
#if !CONFIG_ENABLE_MATTER_OVER_THREAD
            if (!gModemSleepEnabled) {
                esp_wifi_set_ps(WIFI_PS_MIN_MODEM);
                gModemSleepEnabled = true;
                Serial.println("WiFi modem sleep enabled");
            }
#endif
            gState = State::OPERATIONAL;
            break;
        }
        static uint32_t lastPrint = 0;
        if (millis() - lastPrint >= 5000) {
            lastPrint = millis();
#if CONFIG_ENABLE_MATTER_OVER_THREAD
            Serial.println("Waiting for Thread network connection…");
#else
            Serial.println("Waiting for WiFi connection…");
#endif
        }
        break;
    }

    // ── OPERATIONAL ──────────────────────────────────────────────────────────
    // Fully connected. Matter commands are handled via the attributeChangeCB
    // in MatterValve — no polling needed here. Just monitor connectivity.
    case State::OPERATIONAL: {
        // Detect network loss — transition back to CONNECTING.
        if (!MatterInit::isConnected()) {
            Serial.println("Network lost — re-connecting");
            gModemSleepEnabled = false;
            gState = State::CONNECTING;
            break;
        }
        // Detect fabric removal (decommission from controller side).
        if (!MatterInit::isCommissioned()) {
            Serial.println("All fabrics removed — re-commissioning");
            gValveCtrl.allOff();
            gState = State::NOT_COMMISSIONED;
            break;
        }
        // Normal operation: Matter commands arrive asynchronously via callbacks.
        // Nothing to poll here — the CHIP stack runs on its own FreeRTOS task.
        break;
    }

    // ── DECOMMISSIONING ──────────────────────────────────────────────────────
    // Button was held for DECOMMISSION_HOLD_MS. Close everything, factory reset.
    case State::DECOMMISSIONING: {
        Serial.println("ACTION BUTTON: held — decommissioning…");
        gValveCtrl.allOff();
        delay(200);   // brief delay so valve relay de-energizes before reset
        MatterInit::decommission();
        // decommission() calls esp_matter::factory_reset() which restarts the chip.
        // Execution does not reach here — reset takes place immediately.
        while (true) delay(1000);
    }

    }  // end switch

    updateLED(gState);
}
