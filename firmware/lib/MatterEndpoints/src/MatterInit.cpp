// MatterInit.cpp — Matter node lifecycle (no Matter.h dependency)
// Adapted from matter-plant-sensor; ICD/battery features removed.

#include <sdkconfig.h>
#ifdef CONFIG_ESP_MATTER_ENABLE_DATA_MODEL

#include "MatterInit.h"
#include "MatterDeviceProvider.h"
#include <app/server/Server.h>
#include <platform/ConnectivityManager.h>
#include <platform/CHIPDeviceLayer.h>

using namespace esp_matter;
using namespace esp_matter::endpoint;
using namespace esp_matter::cluster;
using namespace esp_matter::identification;

// ── Static member storage ────────────────────────────────────────────────────
bool MatterInit::_initialized      = false;
bool MatterInit::_started          = false;
esp_matter::node_t *MatterInit::_node = nullptr;
std::atomic<bool> MatterInit::_justCommissioned{false};

// ── Event callback ───────────────────────────────────────────────────────────
void MatterInit::eventCB(const ChipDeviceEvent *event, intptr_t /*arg*/) {
    switch (event->Type) {
        case chip::DeviceLayer::DeviceEventType::kCommissioningComplete:
            log_i("MatterInit: commissioning complete");
            _justCommissioned.store(true);
            break;

        case chip::DeviceLayer::DeviceEventType::kFabricRemoved: {
            const chip::FabricTable &ft = chip::Server::GetInstance().GetFabricTable();
            log_i("MatterInit: fabric removed — %u fabric(s) remaining", ft.FabricCount());
            if (ft.FabricCount() == 0) {
                // No fabrics left — re-open commissioning window so the device
                // can be re-commissioned without a physical factory reset.
                chip::CommissioningWindowManager &mgr =
                    chip::Server::GetInstance().GetCommissioningWindowManager();
                constexpr auto kTimeout = chip::System::Clock::Seconds16(300);
                if (!mgr.IsCommissioningWindowOpen()) {
                    log_i("MatterInit: re-opening commissioning window");
                    mgr.OpenBasicCommissioningWindow(
                        kTimeout, chip::CommissioningWindowAdvertisement::kDnssdOnly);
                }
            }
            break;
        }

        case chip::DeviceLayer::DeviceEventType::kBLEDeinitialized:
            log_d("MatterInit: BLE deinitialized, memory reclaimed");
            break;

        default:
            break;
    }
}

// ── Attribute-update callback ────────────────────────────────────────────────
esp_err_t MatterInit::attrUpdateCB(
    attribute::callback_type_t type,
    uint16_t endpoint_id, uint32_t cluster_id, uint32_t attribute_id,
    esp_matter_attr_val_t *val, void *priv_data)
{
    if (type == attribute::PRE_UPDATE && priv_data != nullptr) {
        MatterEndPoint *ep = static_cast<MatterEndPoint *>(priv_data);
        return ep->attributeChangeCB(endpoint_id, cluster_id, attribute_id, val)
               ? ESP_OK : ESP_FAIL;
    }
    return ESP_OK;
}

// ── Identify callback ────────────────────────────────────────────────────────
esp_err_t MatterInit::identifyCB(
    identification::callback_type_t type,
    uint16_t endpoint_id, uint8_t effect_id, uint8_t effect_variant,
    void *priv_data)
{
    if (priv_data != nullptr) {
        MatterEndPoint *ep = static_cast<MatterEndPoint *>(priv_data);
        bool active = (type == identification::START);
        return ep->endpointIdentifyCB(endpoint_id, active) ? ESP_OK : ESP_FAIL;
    }
    return ESP_OK;
}

// ── getAndClearJustCommissioned() ────────────────────────────────────────────
bool MatterInit::getAndClearJustCommissioned() {
    return _justCommissioned.exchange(false);
}

// ── init() ───────────────────────────────────────────────────────────────────
bool MatterInit::init() {
    if (_initialized) return true;

    static node::config_t node_config;
    _node = node::create(&node_config, attrUpdateCB, identifyCB);
    if (_node == nullptr) {
        log_e("MatterInit: failed to create Matter node");
        return false;
    }

    _initialized = true;
    log_i("MatterInit: node initialized");
    return true;
}

// ── start() ──────────────────────────────────────────────────────────────────
bool MatterInit::start() {
    if (!_initialized) {
        log_e("MatterInit: call init() before start()");
        return false;
    }
    if (_started) return true;

#if CONFIG_ENABLE_MATTER_OVER_THREAD
    esp_openthread_platform_config_t ot_config;
    memset(&ot_config, 0, sizeof(ot_config));
    ot_config.radio_config.radio_mode          = RADIO_MODE_NATIVE;
    ot_config.host_config.host_connection_mode = HOST_CONNECTION_MODE_NONE;
    ot_config.port_config.storage_partition_name = "nvs";
    ot_config.port_config.netif_queue_size     = 10;
    ot_config.port_config.task_queue_size      = 10;
    set_openthread_platform_config(&ot_config);
#endif

    // Device identity and setup codes from the MATTER_DEVICE_* build flags.
    // Must happen before esp_matter::start(), which opens the commissioning window.
    MatterDeviceProvider::install();

    esp_err_t err = esp_matter::start(eventCB);
    if (err != ESP_OK) {
        log_e("MatterInit: esp_matter::start() failed: %d", err);
        return false;
    }
    _started = true;
    log_i("MatterInit: Matter stack started");
    return true;
}

// ── Status queries ───────────────────────────────────────────────────────────
bool MatterInit::isCommissioned() {
    return chip::Server::GetInstance().GetFabricTable().FabricCount() > 0;
}

bool MatterInit::isConnected() {
#if CONFIG_ENABLE_MATTER_OVER_THREAD
    return chip::DeviceLayer::ConnectivityMgr().IsThreadAttached();
#else
    return chip::DeviceLayer::ConnectivityMgr().IsWiFiStationConnected();
#endif
}

bool MatterInit::isCommissioningWindowOpen() {
    return chip::Server::GetInstance().GetCommissioningWindowManager()
               .IsCommissioningWindowOpen();
}

void MatterInit::decommission() {
    log_i("MatterInit: factory reset");
    esp_matter::factory_reset();
}

#endif // CONFIG_ESP_MATTER_ENABLE_DATA_MODEL
