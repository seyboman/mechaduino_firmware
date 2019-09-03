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
 * @brief       Mechaduino shell commands
 *
 * @author      Florian Seybold <florian@seybold.space>
 */

#ifndef MECHADUINO_COMMANDS_H
#define MECHADUINO_COMMANDS_H

#include <shell.h>

int calibrate_cmd_handler(int argc, char **argv);
int walk_cmd_handler(int argc, char **argv);

static const shell_command_t mechaduino_commands[] = {
  { "calibrate", "calibrate rotary encoder", calibrate_cmd_handler },
  { "walk", "let stepper walk away", walk_cmd_handler },
  { NULL, NULL, NULL }
};

#endif /* MECHADUINO_COMMANDS_H */
/** @} */
