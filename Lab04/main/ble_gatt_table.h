/*
 * SPDX-FileCopyrightText: 2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */


#include <stdio.h>
#include <stdlib.h>
#include <string.h>


/* Attributes State Machine */
enum
{
    IDX_SVC,
    IDX_CHAR_MQTT_CHAT_DECLARE,
    IDX_CHAR_MQTT_CHAT_VALUE,
    IDX_CHAR_MQTT_CHAT_DESC,
    IDX_CHAR_MQTT_CHAT_CLIENT_CONFIG,

    HRS_IDX_NB,
};
