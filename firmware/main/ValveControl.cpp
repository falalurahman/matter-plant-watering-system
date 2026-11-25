// ValveControl.cpp — GPIO control for solenoid valves and pump motor

#include "ValveControl.h"

// ── _setPin() ────────────────────────────────────────────────────────────────
// Writes a logical level to a GPIO, inverting if RELAY_ACTIVE_LOW=1.
void ValveControl::_setPin(uint8_t pin, bool active) {
#if RELAY_ACTIVE_LOW
    digitalWrite(pin, active ? LOW : HIGH);
#else
    digitalWrite(pin, active ? HIGH : LOW);
#endif
}

// ── begin() ──────────────────────────────────────────────────────────────────
void ValveControl::begin() {
    for (uint8_t i = 0; i < NUM_VALVES; i++) {
        pinMode(VALVE_PINS[i], OUTPUT);
        _setPin(VALVE_PINS[i], false);  // all valves closed
        _valveStates[i] = false;
    }
    pinMode(MOTOR_PIN, OUTPUT);
    _setPin(MOTOR_PIN, false);          // motor off
}

// ── setValve() ───────────────────────────────────────────────────────────────
void ValveControl::setValve(uint8_t index, bool open) {
    if (index >= NUM_VALVES) {
        log_e("ValveControl: index %u out of range", index);
        return;
    }
    _valveStates[index] = open;
    _setPin(VALVE_PINS[index], open);
    log_i("ValveControl: valve[%u] → %s", index, open ? "OPEN" : "CLOSED");
    _updateMotor();
}

// ── isValveOpen() ────────────────────────────────────────────────────────────
bool ValveControl::isValveOpen(uint8_t index) const {
    if (index >= NUM_VALVES) return false;
    return _valveStates[index];
}

// ── isAnyValveOpen() ─────────────────────────────────────────────────────────
bool ValveControl::isAnyValveOpen() const {
    for (uint8_t i = 0; i < NUM_VALVES; i++) {
        if (_valveStates[i]) return true;
    }
    return false;
}

// ── allOff() ─────────────────────────────────────────────────────────────────
void ValveControl::allOff() {
    for (uint8_t i = 0; i < NUM_VALVES; i++) {
        _valveStates[i] = false;
        _setPin(VALVE_PINS[i], false);
    }
    _setPin(MOTOR_PIN, false);
    log_i("ValveControl: all valves closed, motor OFF");
}

// ── _updateMotor() ───────────────────────────────────────────────────────────
void ValveControl::_updateMotor() {
    bool motorOn = isAnyValveOpen();
    _setPin(MOTOR_PIN, motorOn);
    log_d("ValveControl: motor → %s", motorOn ? "ON" : "OFF");
}
