#include "ble.h"

static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA_BYTES(BT_DATA_UUID16_ALL, 0x12, 0x18, /* HID Service */
                  0x0f, 0x18),                    /* Battery Service */
};

static struct bt_conn_cb conn_callbacks = {
    .connected = connected,
    .disconnected = disconnected,
    .security_changed = security_changed,
};

static struct bt_conn_auth_cb auth_cb_display = {
    .passkey_display = auth_passkey_display,
    .passkey_entry = NULL,
    .cancel = auth_cancel,
};

void connected(struct bt_conn *conn, u8_t err) {
  char addr[BT_ADDR_LE_STR_LEN];

  bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

  if (err) {
    printk("Failed to connect to %s (%u)\n", addr, err);
    return;
  }

  printk("Connected %s\n", addr);

  if (bt_conn_set_security(conn, BT_SECURITY_L2)) {
    printk("Failed to set security\n");
  }
}

void disconnected(struct bt_conn *conn, u8_t reason) {
  char addr[BT_ADDR_LE_STR_LEN];

  bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

  printk("Disconnected from %s (reason 0x%02x)\n", addr, reason);
}

void security_changed(struct bt_conn *conn, bt_security_t level,
                      enum bt_security_err err) {
  char addr[BT_ADDR_LE_STR_LEN];

  bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

  if (!err) {
    printk("Security changed: %s level %u", addr, level);
  } else {
    printk("Security failed: %s level %u err %d", addr, level, err);
  }
}

void bt_ready(int err) {
  if (err) {
    printk("Bluetooth init failed (err %d)\n", err);
    return;
  }

  printk("Bluetooth initialized\n");

  if (IS_ENABLED(CONFIG_SETTINGS)) {
    printk("Sam3\n");
    settings_load();
  }

  err = bt_le_adv_start(BT_LE_ADV_CONN_NAME, ad, ARRAY_SIZE(ad), NULL, 0);
  if (err) {
    printk("Advertising failed to start (err %d)\n", err);
    return;
  }

  printk("Advertising successfully started\n");
}

void auth_passkey_display(struct bt_conn *conn, unsigned int passkey) {
  char addr[BT_ADDR_LE_STR_LEN];

  bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

  printk("Passkey for %s: %06u\n", addr, passkey);
}

void auth_cancel(struct bt_conn *conn) {
  char addr[BT_ADDR_LE_STR_LEN];

  bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

  printk("Pairing cancelled: %s\n", addr);
}

void ble_init() {
  int err;
  struct bt_le_conn_param param;

  printk("Sam1\n");
  err = bt_enable(bt_ready);
  if (err) {
    printk("Bluetooth init failed (err %d)\n", err);
    return;
  }
  printk("Sam2\n");

  param.interval_min = 10;
  param.interval_max = 50;
  param.latency = 0;
  param.timeout = 400;

  err = bt_conn_le_param_update(NULL, &param);

  bt_conn_cb_register(&conn_callbacks);
  bt_conn_auth_cb_register(&auth_cb_display);
}
