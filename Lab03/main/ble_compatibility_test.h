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
    IDX_CHAR_MEM_1_DECLARE,
    IDX_CHAR_MEM_1_VALUE,
    IDX_CHAR_MEM_1_DESC,
    IDX_CHAR_MEM_1_FORMAT,

    IDX_CHAR_MEM_2_DECLARE,
    IDX_CHAR_MEM_2_VALUE,
    IDX_CHAR_MEM_2_DESC,
    IDX_CHAR_MEM_2_FORMAT,

    IDX_CHAR_MEM_3_DECLARE,
    IDX_CHAR_MEM_3_VALUE,
    IDX_CHAR_MEM_3_DESC,
    IDX_CHAR_MEM_3_FORMAT,

    HRS_IDX_NB,
};
