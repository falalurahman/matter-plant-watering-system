// MatterValve.cpp — on_off_plugin_unit endpoint for solenoid valve control

#include <sdkconfig.h>
#ifdef CONFIG_ESP_MATTER_ENABLE_DATA_MODEL

#include "MatterValve.h"
#include "MatterInit.h"

using namespace esp_matter;
using namespace esp_matter::endpoint;
using namespace chip::app::Clusters;

// ── attributeChangeCB ────────────────────────────────────────────────────────
// Called by MatterInit::attrUpdateCB (PRE_UPDATE) when a controller changes
// the OnOff attribute. We read the new value and fire the onChange callback.
bool MatterValve::attributeChangeCB(uint16_t endpoint_id, uint32_t cluster_id,
                                     uint32_t attribute_id,
                                     esp_matter_attr_val_t *val) {
    if (!_started) return false;

    if (cluster_id == OnOff::Id && attribute_id == OnOff::Attributes::OnOff::Id) {
        bool newState = val->val.b;
        log_i("MatterValve[%u]: OnOff → %s (ep=%u)", _valveIndex,
              newState ? "ON" : "OFF", endpoint_id);
        _state = newState;
        if (onChange) {
            onChange(_valveIndex, newState);
        }
    }
    return true;
}

// ── begin() ──────────────────────────────────────────────────────────────────
bool MatterValve::begin(uint8_t valveIndex, bool initialState) {
    if (getEndPointId() != 0) {
        log_e("MatterValve[%u]: endpoint already created (id=%d)", valveIndex, getEndPointId());
        return false;
    }

    _valveIndex = valveIndex;
    _state      = initialState;

    on_off_plugin_unit::config_t cfg;
    cfg.on_off.on_off                     = initialState;
    cfg.on_off.lighting.start_up_on_off   = nullptr;

    endpoint_t *ep = on_off_plugin_unit::create(node::get(), &cfg, ENDPOINT_FLAG_NONE,
                                                 static_cast<void *>(this));
    if (ep == nullptr) {
        log_e("MatterValve[%u]: failed to create endpoint", valveIndex);
        return false;
    }

    setEndPointId(endpoint::get_id(ep));
    log_i("MatterValve[%u]: endpoint_id=%d", valveIndex, getEndPointId());
    _started = true;
    return true;
}

void MatterValve::end() {
    _started = false;
}

// ── setState() ───────────────────────────────────────────────────────────────
// Pushes a state change to the Matter attribute (reports to subscribed controllers).
bool MatterValve::setState(bool on) {
    if (!_started) {
        log_e("MatterValve[%u]: not started", _valveIndex);
        return false;
    }
    if (_state == on) return true;

    esp_matter_attr_val_t val = esp_matter_invalid(NULL);
    if (!getAttributeVal(OnOff::Id, OnOff::Attributes::OnOff::Id, &val)) {
        log_e("MatterValve[%u]: failed to read OnOff attribute", _valveIndex);
        return false;
    }
    val.val.b = on;
    if (!updateAttributeVal(OnOff::Id, OnOff::Attributes::OnOff::Id, &val)) {
        log_e("MatterValve[%u]: failed to update OnOff attribute", _valveIndex);
        return false;
    }
    _state = on;
    log_v("MatterValve[%u]: state → %s", _valveIndex, on ? "ON" : "OFF");
    return true;
}

#endif // CONFIG_ESP_MATTER_ENABLE_DATA_MODEL
