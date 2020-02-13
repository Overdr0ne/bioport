#ifndef __BLE_H_
#define __BLE_H_

#include <bluetooth/bluetooth.h>
#include <bluetooth/conn.h>
#include <bluetooth/gatt.h>
#include <bluetooth/hci.h>
#include <bluetooth/uuid.h>

#include <settings/settings.h>

void connected(struct bt_conn *conn, u8_t err);
void disconnected(struct bt_conn *conn, u8_t reason);
void security_changed(struct bt_conn *conn, bt_security_t level,
                      enum bt_security_err err);
void bt_ready(int err);
void auth_passkey_display(struct bt_conn *conn, unsigned int passkey);
void auth_cancel(struct bt_conn *conn);

void ble_init();

#endif // __BLE_H_
