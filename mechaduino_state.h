/*
 * Copyright (C) 2019 Florian Seybold
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     mechaduino_firmware
 *
 * @{
 * @file
 * @brief       Mechaduino state
 *
 * @author      Florian Seybold <florian@seybold.space>
 */

#ifndef MECHADUINO_STATE_H
#define MECHADUINO_STATE_H

#include <as5047d.h>

extern as5047d_t enc_dev;

extern int dir;
extern int stepNumber; // step index for cal routine

#endif
