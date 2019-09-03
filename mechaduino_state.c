/*
 * Copyright (C) 2019 Florian Seybold
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

#include "mechaduino_state.h"

volatile as5047d_t enc_dev;

volatile int dir = false;
volatile int stepNumber = 0; // step index for cal routine
