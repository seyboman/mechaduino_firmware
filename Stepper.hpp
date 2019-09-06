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
 * @brief       Mechaduino stepper
 *
 * @author      Florian Seybold <florian@seybold.space>
 */

#ifndef STEPPER_HPP
#define STEPPER_HPP

#include <xtimer.h>

#include "Motor.hpp"

class Stepper {
public:
   Stepper(const Motor& motor_, const bool& dir_ = true)
      :  motor(motor_),
         dir(dir_),
         stepNumber(0)
   { }

   void step()
   {
      if (!dir) {
         stepNumber += 1;
      }
      else {
         stepNumber -= 1;
      }

      //output(1.8 * stepNumber, 64); //updata 1.8 to aps..., second number is control effort
      motor.output(motor.aps * stepNumber, (int)(0.33 * motor.uMax));
      xtimer_usleep(10000);
   }

   int dir;
   int stepNumber; // step index for cal routine
   const Motor& motor;

private:

};

#endif
