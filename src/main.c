/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

/** @file
 *  @brief Nordic UART Service Client sample
 */

#include <errno.h>
#include <zephyr.h>
#include <sys/byteorder.h>
#include <sys/printk.h>

#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/conn.h>
#include <bluetooth/uuid.h>
#include <bluetooth/gatt.h>

#include <bluetooth/services/nus.h>
#include <bluetooth/services/nus_client.h>
#include <bluetooth/gatt_dm.h>
#include <bluetooth/scan.h>

#include <settings/settings.h>

#include <drivers/uart.h>

#include <logging/log.h>
#include <dk_buttons_and_leds.h>

#define LOG_MODULE_NAME central_uart
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

/* UART payload buffer element size. */
#define UART_BUF_SIZE 20

#define KEY_PASSKEY_ACCEPT DK_BTN1_MSK
#define KEY_PASSKEY_REJECT DK_BTN2_MSK

#define NUS_WRITE_TIMEOUT K_MSEC(150)
#define UART_WAIT_FOR_BUF_DELAY K_MSEC(50)
#define UART_RX_TIMEOUT 50

static struct bt_conn *default_conn;
static struct bt_nus_client nus_client;

static void button_handler_cb(uint32_t button_state, uint32_t has_changed);

static struct button_handler button = {
    .cb = button_handler_cb,
};

static int scan_param_set(void);
static int event_scan(bool connect);
static void event_disconnect(void);
static int event_scan_stop(void);

static void button_handler_cb(uint32_t button_state, uint32_t has_changed)
{
    uint32_t buttons = button_state & has_changed;

    if (buttons & DK_BTN1_MSK) {
        printk("\nScan only\n");
        event_scan(false);
    } else if (buttons & DK_BTN2_MSK) {
        printk("\nStop scan\n");
        event_scan_stop();
    } else if (buttons & DK_BTN3_MSK) {
        printk("\nScan and connect\n");
        event_scan(true);
    } else if (buttons & DK_BTN4_MSK) {
        printk("\nDisconnect\n");
        event_disconnect();
    } else {
        return;
    }
}

static void buttons_init(void)
{
    int err;

    err = dk_buttons_init(NULL);
    if (err) {
        printk("Buttons initialization failed.\n");
        return;
    }

    /* Add dynamic buttons handler. Buttons should be activated only when
     * during the board role choosing.
     */
    dk_button_handler_add(&button);
}

static int event_scan(bool connect)
{
    int err = 0;
    struct bt_scan_init_param scan_init = {
        .connect_if_match = 1,
    };

    if (connect) {
        scan_init.connect_if_match = 1;
    } else {
        scan_init.connect_if_match = 0;
    }

    bt_scan_init(&scan_init);

    err = scan_param_set();
    if (err) {
        printk("Scan falied\n\r");
        return err;
    }

    err = bt_scan_start(BT_SCAN_TYPE_SCAN_ACTIVE);
    if (err) {
        printk("Scanning failed to start (err %d)\n\r", err);
        return err;
    }

    printk("Scan started\n\r");

    return err;
}

static void event_disconnect(void)
{
    int ret = bt_conn_disconnect(default_conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
    if (ret != 0) {
        printk("disconnect error %d\n\r", ret);
    }
}

static int event_scan_stop(void)
{
    int err;

    err = bt_scan_stop();
    if (err != 0) {
        printk("Stop LE scan failed (err %d)\n\r", err);
    }
    printk("Scanning stopped\n\r");

    return err;
}

static void ble_data_sent(uint8_t err, const uint8_t *const data, uint16_t len)
{

}

static uint8_t ble_data_received(const uint8_t *const data, uint16_t len)
{
    return BT_GATT_ITER_CONTINUE;
}


static void discovery_complete(struct bt_gatt_dm *dm, void *context)
{
    struct bt_nus_client *nus = context;
    printk("Service discovery completed\n\r");

    bt_gatt_dm_data_print(dm);

    bt_nus_handles_assign(dm, nus);
    bt_nus_subscribe_receive(nus);

    bt_gatt_dm_data_release(dm);
}

static void discovery_service_not_found(struct bt_conn *conn, void *context)
{
    printk("Service not found\n\r");
}

static void discovery_error(struct bt_conn *conn, int err, void *context)
{
    printk("Error discovering GATT database: (%d)\n\r", err);
}

struct bt_gatt_dm_cb discovery_cb = {
    .completed         = discovery_complete,
    .service_not_found = discovery_service_not_found,
    .error_found       = discovery_error,
};

static void gatt_discover(struct bt_conn *conn)
{
    int err;

    if (conn != default_conn) {
        return;
    }

    err = bt_gatt_dm_start(conn,
                   BT_UUID_NUS_SERVICE,
                   &discovery_cb,
                   &nus_client);
    if (err) {
        printk("could not start discovery procedure " "code: %d\n\r", err);
    }
}

static void connected(struct bt_conn *conn, uint8_t conn_err)
{
    char addr[BT_ADDR_LE_STR_LEN];
    int err;

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    if (conn_err) {
        printk("Failed to connect to %s (%d)\n\r", log_strdup(addr), conn_err);

        if (default_conn == conn) {
            bt_conn_unref(default_conn);
            default_conn = NULL;

            err = bt_scan_start(BT_SCAN_TYPE_SCAN_ACTIVE);
            if (err) {
                printk("Scanning failed to start (err %d)\n\r", err);
            }
        }

        return;
    }

    printk("Connected: %s\n\r", log_strdup(addr));

    err = bt_conn_set_security(conn, BT_SECURITY_L2);
    if (err) {
        printk("Failed to set security: %d\n\r", err);

        gatt_discover(conn);
    }

    err = bt_scan_stop();
    if ((!err) && (err != -EALREADY)) {
        printk("Stop LE scan failed (err %d)\n\r", err);
    }
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
    char addr[BT_ADDR_LE_STR_LEN];
    //int err;

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    printk("Disconnected: %s (reason %u)\n\r", log_strdup(addr),
        reason);

    if (default_conn != conn) {
        return;
    }

    bt_conn_unref(default_conn);
    default_conn = NULL;

    /*err = bt_scan_start(BT_SCAN_TYPE_SCAN_ACTIVE);
    if (err) {
        printk("Scanning failed to start (err %d)\n\r", err);
    }*/
}

static void security_changed(struct bt_conn *conn, bt_security_t level,
                 enum bt_security_err err)
{
    char addr[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    if (!err) {
        printk("Security changed: %s level %u\n\r", log_strdup(addr), level);
    } else {
        printk("Security failed: %s level %u err %d\n\r", log_strdup(addr),
            level, err);
    }

    gatt_discover(conn);
}

static struct bt_conn_cb conn_callbacks = {
    .connected = connected,
    .disconnected = disconnected,
    .security_changed = security_changed
};

static void scan_filter_match(struct bt_scan_device_info *device_info,
                  struct bt_scan_filter_match *filter_match,
                  bool connectable)
{
    printk("Beacon -> ");
    for (uint8_t i = 0; i < 6; i++) {
        printk("%02x ",device_info->recv_info->addr->a.val[i]);
    }
    printk(": RSSI %d\n\r", device_info->recv_info->rssi);
}

static void scan_connecting_error(struct bt_scan_device_info *device_info)
{
    printk("Connecting failed\n\r");
}

static void scan_connecting(struct bt_scan_device_info *device_info,
                struct bt_conn *conn)
{
    default_conn = bt_conn_ref(conn);
}

static int nus_client_init(void)
{
    int err;
    struct bt_nus_client_init_param init = {
        .cb = {
            .received = ble_data_received,
            .sent = ble_data_sent,
        }
    };

    err = bt_nus_client_init(&nus_client, &init);
    if (err) {
        printk("NUS Client initialization failed (err %d)\n\r", err);
        return err;
    }

    printk("NUS Client module initialized\n\r");
    return err;
}

BT_SCAN_CB_INIT(scan_cb, scan_filter_match, NULL,
        scan_connecting_error, scan_connecting);

static int scan_ble_init(void)
{
    int err = 0;

    bt_scan_cb_register(&scan_cb);

    printk("Scan module initialized\n\r");
    return err;
}

static int scan_param_set(void)
{
    int err = 0;

    err = bt_scan_filter_add(BT_SCAN_FILTER_TYPE_UUID, BT_UUID_NUS_SERVICE);
    if (err) {
        printk("Scanning filters cannot be set (err %d)\n\r", err);
        return err;
    }

    err = bt_scan_filter_enable(BT_SCAN_UUID_FILTER, false);
    if (err) {
        printk("Filters cannot be turned on (err %d)\n\r", err);
        return err;
    }

    return err;
}

static void auth_cancel(struct bt_conn *conn)
{
    char addr[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    printk("Pairing cancelled: %s\n\r", log_strdup(addr));
}

static void pairing_confirm(struct bt_conn *conn)
{
    char addr[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    bt_conn_auth_pairing_confirm(conn);

    printk("Pairing confirmed: %s\n\r", log_strdup(addr));
}

static void pairing_complete(struct bt_conn *conn, bool bonded)
{
    char addr[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    printk("Pairing completed: %s, bonded: %d\n\r", log_strdup(addr),
        bonded);
}

static void pairing_failed(struct bt_conn *conn, enum bt_security_err reason)
{
    char addr[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    printk("Pairing failed conn: %s, reason %d\n\r", log_strdup(addr),
        reason);
}

static struct bt_conn_auth_cb conn_auth_callbacks = {
    .cancel = auth_cancel,
    .pairing_confirm = pairing_confirm,
    .pairing_complete = pairing_complete,
    .pairing_failed = pairing_failed
};

static void bt_ready(int err)
{
    if (err) {
        printk("BLE failed %d\r\n", err);
        return;
    }
    printk("Bluetooth initialized\n\r");

    if (IS_ENABLED(CONFIG_SETTINGS)) {
        settings_load();
    }

    bt_conn_cb_register(&conn_callbacks);

    int (*module_init[])(void) = {scan_ble_init, nus_client_init};
    for (size_t i = 0; i < ARRAY_SIZE(module_init); i++) {
        err = (*module_init[i])();
        if (err) {
            printk("failed func %d\n\r",i);
            return;
        }
    }

    printk("Starting Bluetooth Central UART example\n\r");

    buttons_init();
}

void main(void)
{
    int err;

    err = bt_conn_auth_cb_register(&conn_auth_callbacks);
    if (err) {
        printk("Failed to register authorization callbacks\n\r");
        return;
    }

    err = bt_enable(bt_ready);
    if (err) {
        printk("Bluetooth init failed (err %d)\n\r", err);
        return;
    }
}
