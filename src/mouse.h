/** @file
 *  @brief mouse Service sample
 */

/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifdef __cplusplus
extern "C" {
#endif

#define MOUSE_BTN_REPORT_POS 0
#define MOUSE_X_REPORT_POS 0
#define MOUSE_Y_REPORT_POS 1

#define MOUSE_BTN_LEFT BIT(0)
#define MOUSE_BTN_RIGHT BIT(1)
#define MOUSE_BTN_MIDDLE BIT(2)

struct mouse_status {
  u8_t button;
  s16_t pos[2];
};

void mouse_init(void);
int mouse_notify(struct mouse_status *status);

#ifdef __cplusplus
}
#endif
