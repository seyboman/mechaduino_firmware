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

#include "as5047d_params.h"

#include "mechaduino_state.h"
#include "mechaduino_commands.h"


char rclc_thread_stack[THREAD_STACKSIZE_MAIN];

void *rclc_thread(void *arg)
{
  int16_t angle = -1;
  as5047d_t dev;
  if (as5047d_init(&dev, &as5047d_params[0])) {
      puts("[Init of as5047d failed]");
  }

  static int argc = 0;
  static char **argv = NULL;

  rclc_init(argc, argv);
  rclc_node_t* node = rclc_create_node("talker", "");
  rclc_publisher_t* pub = rclc_create_publisher(node, RCLC_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "chatter", 1);

  std_msgs__msg__String msg;
  char buff[64] = {0};
  msg.data.data = buff;
  msg.data.capacity = sizeof(buff);
  msg.data.size = 0;
  
  int i = 1;

  while (rclc_ok()) {
    angle = as5047d_read(&dev);
    msg.data.size = snprintf(msg.data.data, msg.data.capacity, "Rotary encoder: %i", angle);
    if(msg.data.size > msg.data.capacity) msg.data.size = 0;

    //if(msg.data.data[msg.data.size] == '\0') {
    /*if(msg.data.data[msg.data.size] == '\0' && i%100==0) {
      printf("Publishing: '%s'\n", msg.data.data);
    }*/
    
    rclc_publish(pub, (const void*)&msg);

    rclc_spin_node_once(node, 100);
  }
    
  rclc_destroy_publisher(pub);
  rclc_destroy_node(node);
}

int main(void)
{
  init_params();

  /*
  thread_create(rclc_thread_stack, sizeof(rclc_thread_stack),
                    THREAD_PRIORITY_MAIN + 1,
                    THREAD_CREATE_STACKTEST,
                    rclc_thread,
                    NULL, "rclc_thread");
  */

  if (as5047d_init(&enc_dev, &as5047d_params[0])) {
      puts("[Init of as5047d failed]");
  }

  /* start shell */
  puts("Starting the shell now...");
  char line_buf[SHELL_DEFAULT_BUFSIZE];
  shell_run(mechaduino_commands, line_buf, SHELL_DEFAULT_BUFSIZE);
  /* start shell */
}

