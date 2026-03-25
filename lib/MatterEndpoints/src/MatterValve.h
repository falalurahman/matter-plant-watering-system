// MatterValve.h — on_off_plugin_unit endpoint for solenoid valve control
// One instance per valve (10 total). Does NOT include Matter.h.

#pragma once
#include <sdkconfig.h>
#ifdef CONFIG_ESP_MATTER_ENABLE_DATA_MODEL

#include <MatterEndPoint.h>
#include <functional>

class MatterValve : public MatterEndPoint {
public:
    MatterValve() {}
    ~MatterValve() { end(); }

    // Initialize and register the endpoint.
    // valveIndex: 0-based index used to identify this valve in onChange callbacks.
    // initialState: true = open/on, false = closed/off.
    bool begin(uint8_t valveIndex, bool initialState = false);
    void end();

    // Update the on/off state of this valve (pushes the Matter attribute).
    bool setState(bool on);
    bool getState() const { return _state; }

    // Convenience operators
    void operator=(bool on) { setState(on); }
    operator bool()         { return getState(); }

    // Called by MatterInit::attrUpdateCB when a Matter controller changes
    // the OnOff attribute on this endpoint (e.g. "turn on valve 3").
    bool attributeChangeCB(uint16_t endpoint_id, uint32_t cluster_id,
                           uint32_t attribute_id,
                           esp_matter_attr_val_t *val) override;

    // Callback invoked on this endpoint's thread whenever the state changes
    // due to a Matter command. Set this from main.cpp to wire valve → GPIO.
    // Signature: void(uint8_t valveIndex, bool on)
    std::function<void(uint8_t, bool)> onChange;

private:
    uint8_t _valveIndex = 0;
    bool    _state      = false;
    bool    _started    = false;
};

#endif // CONFIG_ESP_MATTER_ENABLE_DATA_MODEL
