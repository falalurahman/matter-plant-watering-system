// ValveControl.h — GPIO control for 10 solenoid valves + 1 motor relay/MOSFET
//
// GPIO polarity is controlled by the RELAY_ACTIVE_LOW build flag:
//   RELAY_ACTIVE_LOW=0  →  GPIO HIGH = device ON  (N-channel MOSFETs, active-high relays)
//   RELAY_ACTIVE_LOW=1  →  GPIO LOW  = device ON  (optocoupler relay boards, active-low)
//
// Motor logic: motor turns ON when any valve is open; OFF when all are closed.

#pragma once
#include <Arduino.h>

// GPIO pin arrays — resolved from per-board build flags in platformio.ini
constexpr uint8_t NUM_VALVES = 10;
constexpr uint8_t VALVE_PINS[NUM_VALVES] = {
    VALVE_PIN_0, VALVE_PIN_1, VALVE_PIN_2, VALVE_PIN_3, VALVE_PIN_4,
    VALVE_PIN_5, VALVE_PIN_6, VALVE_PIN_7, VALVE_PIN_8, VALVE_PIN_9
};

class ValveControl {
public:
    // Configure all valve and motor GPIO pins as outputs and set them all OFF.
    void begin();

    // Open or close a single valve (0-based index). Motor is updated automatically.
    void setValve(uint8_t index, bool open);

    // Query state
    bool isValveOpen(uint8_t index) const;
    bool isAnyValveOpen() const;

    // Close all valves and stop the motor. Called on decommission and at boot.
    void allOff();

private:
    bool _valveStates[NUM_VALVES] = {};

    // Write a logical active/inactive level to a pin, honouring RELAY_ACTIVE_LOW.
    void _setPin(uint8_t pin, bool active);

    // Re-evaluate motor state: ON if any valve is open, OFF otherwise.
    void _updateMotor();
};
