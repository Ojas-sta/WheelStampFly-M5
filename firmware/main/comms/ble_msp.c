#include "ble_msp.h"
#include "msp.h"
#include "esp_log.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include <string.h>

static const char *TAG = "BLE_MSP";

// Nordic UART Service (NUS) UUIDs for Betaflight Configurator
// 6E400001-B5A3-F393-E0A9-E50E24DCCA9E
static const ble_uuid128_t gatt_svr_svc_uart_uuid =
    BLE_UUID128_INIT(0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0,
                     0x93, 0xf3, 0xa3, 0xb5, 0x01, 0x00, 0x40, 0x6e);

// RX Characteristic: 6E400002-B5A3-F393-E0A9-E50E24DCCA9E (Write from Host)
static const ble_uuid128_t gatt_svr_chr_uart_rx_uuid =
    BLE_UUID128_INIT(0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0,
                     0x93, 0xf3, 0xa3, 0xb5, 0x02, 0x00, 0x40, 0x6e);

// TX Characteristic: 6E400003-B5A3-F393-E0A9-E50E24DCCA9E (Notify to Host)
static const ble_uuid128_t gatt_svr_chr_uart_tx_uuid =
    BLE_UUID128_INIT(0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0,
                     0x93, 0xf3, 0xa3, 0xb5, 0x03, 0x00, 0x40, 0x6e);

static uint16_t tx_val_handle;
static uint16_t conn_handle = BLE_HS_CONN_HANDLE_NONE;
static bool notify_enabled = false;

static int gatt_svr_chr_access_uart(uint16_t conn_h, uint16_t attr_handle,
                                   struct ble_gatt_access_ctxt *ctxt, void *arg);

static const struct ble_gatt_svc_def gatt_svr_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &gatt_svr_svc_uart_uuid.u,
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                // RX (Host Writes to FC)
                .uuid = &gatt_svr_chr_uart_rx_uuid.u,
                .access_cb = gatt_svr_chr_access_uart,
                .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP,
            },
            {
                // TX (FC Notifies Host)
                .uuid = &gatt_svr_chr_uart_tx_uuid.u,
                .access_cb = gatt_svr_chr_access_uart,
                .val_handle = &tx_val_handle,
                .flags = BLE_GATT_CHR_F_NOTIFY,
            },
            { 0 } // No more characteristics
        },
    },
    { 0 } // No more services
};

static int gatt_svr_chr_access_uart(uint16_t conn_h, uint16_t attr_handle,
                                   struct ble_gatt_access_ctxt *ctxt, void *arg) {
    if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
        struct os_mbuf *om = ctxt->om;
        while (om != NULL) {
            for (int i = 0; i < om->om_len; i++) {
                msp_process_byte(om->om_data[i]);
            }
            om = SLIST_NEXT(om, om_next);
        }
        return 0;
    }
    return BLE_ATT_ERR_UNLIKELY;
}

static void ble_advertise(void) {
    struct ble_gap_adv_params adv_params;
    struct ble_hs_adv_fields fields;

    memset(&fields, 0, sizeof(fields));
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;

    const char *device_name = "OsusX";
    fields.name = (uint8_t *)device_name;
    fields.name_len = strlen(device_name);
    fields.name_is_complete = 1;

    ble_gap_adv_set_fields(&fields);

    memset(&adv_params, 0, sizeof(adv_params));
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, NULL, BLE_HS_FOREVER,
                      &adv_params, NULL, NULL);

    ESP_LOGI(TAG, "Bluetooth BLE advertising as 'OsusX' for Betaflight Configurator");
}

static int ble_gap_event(struct ble_gap_event *event, void *arg) {
    switch (event->type) {
        case BLE_GAP_EVENT_CONNECT:
            if (event->connect.status == 0) {
                conn_handle = event->connect.conn_handle;
                notify_enabled = true;
                ESP_LOGI(TAG, "Betaflight Configurator connected over Bluetooth BLE!");
            } else {
                ble_advertise();
            }
            break;

        case BLE_GAP_EVENT_DISCONNECT:
            conn_handle = BLE_HS_CONN_HANDLE_NONE;
            notify_enabled = false;
            ESP_LOGI(TAG, "Bluetooth BLE disconnected. Re-advertising...");
            ble_advertise();
            break;

        case BLE_GAP_EVENT_SUBSCRIBE:
            notify_enabled = event->subscribe.cur_notify;
            break;
    }
    return 0;
}

static void ble_host_task(void *param) {
    nimble_port_run();
    nimble_port_freertos_deinit();
}

void ble_msp_init(void) {
    nimble_port_init();

    ble_svc_gap_init();
    ble_svc_gatt_init();
    ble_svc_gap_device_name_set("OsusX");

    ble_gatts_count_cfg(gatt_svr_svcs);
    ble_gatts_add_svcs(gatt_svr_svcs);

    ble_hs_cfg.sync_cb = ble_advertise;

    // Run NimBLE host task on Core 0 (Auxiliary Core)
    nimble_port_freertos_init(ble_host_task);
    ESP_LOGI(TAG, "NimBLE Bluetooth stack running on Core 0");
}

void ble_msp_send(const uint8_t *data, size_t len) {
    if (conn_handle != BLE_HS_CONN_HANDLE_NONE && notify_enabled && data && len > 0) {
        struct os_mbuf *om = ble_hs_mbuf_from_flat(data, len);
        if (om) {
            ble_gatts_notify_custom(conn_handle, tx_val_handle, om);
        }
    }
}

bool ble_msp_is_connected(void) {
    return (conn_handle != BLE_HS_CONN_HANDLE_NONE);
}
