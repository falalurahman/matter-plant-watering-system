// MatterInit.h
// Minimalist Matter node initializer — bypasses Matter.h and ArduinoMatter.
// Direct SDK access via esp_matter::node::create().
// Adapted from matter-plant-sensor; ICD/battery features removed (device is always powered).

#pragma once
#include <sdkconfig.h>
#ifdef CONFIG_ESP_MATTER_ENABLE_DATA_MODEL

#include <Arduino.h>
#include <esp_matter.h>
#include <MatterEndPoint.h>
#include <atomic>

#if CONFIG_ENABLE_MATTER_OVER_THREAD
#include "esp_openthread_types.h"
#include "platform/ESP32/OpenthreadLauncher.h"
#endif

// ─── MatterInit ──────────────────────────────────────────────────────────────
// Singleton wrapper for esp_matter node lifecycle.
// Call init() before creating any endpoints, then start() after all endpoints
// have been registered to start the Matter/Thread/WiFi stack.
class MatterInit {
public:
    // Create the Matter root node. Must be called exactly once, before any
    // endpoint's begin() method.
    static bool init();

    // Start the Matter stack (OpenThread or WiFi + BLE commissioning).
    // Call after all endpoints have called begin().
    static bool start();

    // ── Status queries (usable after start()) ────────────────────────────────
    static bool isCommissioned();             // fabric table non-empty
    static bool isConnected();                // Thread attached OR WiFi station connected
    static bool isCommissioningWindowOpen();  // BLE/DNS-SD commissioning window active

    // Factory-reset all Matter credentials and restart the device.
    static void decommission();

    // Returns true (and clears the flag) if kCommissioningComplete fired since
    // the last call.
    static bool getAndClearJustCommissioned();

    // Internal callbacks — do not call directly.
    static esp_err_t attrUpdateCB(
        esp_matter::attribute::callback_type_t type,
        uint16_t endpoint_id, uint32_t cluster_id, uint32_t attribute_id,
        esp_matter_attr_val_t *val, void *priv_data);

    static esp_err_t identifyCB(
        esp_matter::identification::callback_type_t type,
        uint16_t endpoint_id, uint8_t effect_id, uint8_t effect_variant,
        void *priv_data);

private:
    static bool _initialized;
    static bool _started;
    static esp_matter::node_t *_node;
    static std::atomic<bool> _justCommissioned;

    static void eventCB(const ChipDeviceEvent *event, intptr_t arg);
};

#endif // CONFIG_ESP_MATTER_ENABLE_DATA_MODEL
