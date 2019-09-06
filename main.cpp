/*
Copyright 2017 INRIA

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/

#include <rclc/rclc.h>
#include <std_msgs/msg/string.h>

#include <stdio.h>
#include <thread.h>
#include <shell.h>

#include "Motor.hpp"
#include "Stepper.hpp"
#include "Encoder.hpp"

//#include "mechaduino_state.h"
//#include "mechaduino_commands.h"
//#include "mechaduino_params.h"

namespace mechaduino {
   Motor motor;
   Stepper stepper(motor);
   Encoder encoder;
}

int main(void)
{
  /* start shell */
  puts("Starting the shell now...");
  const shell_command_t commands[] = {
     { "step", "let stepper take one step", [](int, char**)->int{ mechaduino::stepper.step(); } },
     { "calibrate", "calibrate encoder", [](int, char**)->int{ mechaduino::encoder.calibrate(mechaduino::stepper); } },
     { NULL, NULL, NULL }
  };
  char line_buf[SHELL_DEFAULT_BUFSIZE];
  shell_run(commands, line_buf, SHELL_DEFAULT_BUFSIZE);
}

