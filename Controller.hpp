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
 * @brief       Mechaduino motor
 *
 * @author      Florian Seybold <florian@seybold.space>
 */

#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP

#include <thread.h>
#include <xtimer.h>

#include <cmath>

#include "Motor.hpp"
#include "Encoder.hpp"

#define ENABLE_DEBUG    (0)
#include "debug.h"


class Controller
{
public:
   Controller(const Motor& motor_, const Encoder& encoder_, const char& priority_=0/*, const uint32_t& period_=1000*/)
      : motor(motor_),
        encoder(encoder_),
        priority(priority_)//,
        //period(period_)
   { }

   void start()
   {
      DEBUG("Controller::start(): Creating thread with priority=%i, period=%li...\n", priority, period);

      if(go==true) return;

      kernel_pid_t pid = thread_create(thread_stack, sizeof(thread_stack),
         priority,
         THREAD_CREATE_STACKTEST,
         [](void* arg)->void*{ return ((Controller*)arg)->run(); },
         (void*)this,
         "controller");
      
      go=true;

      DEBUG("Controller::start(): Created thread %i...\n", pid);
   }

   void stop()
   {
      go=false;
   }

   float r = 0.0; // Setpoint

private:
   void* run()
   {
      DEBUG("Controller::run(): Entering...\n");

      wrap_count = 0;
      y_1 = 0.0;
      yw_1 = 0.0;
      r = 0.0;
      ITerm = 0.0;
      DTerm = 0.0;

      //size_t s = 0;

      last_wakeup=xtimer_now();
      while(go)
      {
         xtimer_periodic_wakeup(&last_wakeup, period);

         float y = encoder.angle();                    //read encoder and lookup corrected angle in calibration lookup table
         if ((y - y_1) < -180.0) wrap_count += 1;      //Check if we've rotated more than a full revolution (have we "wrapped" around from 359 degrees to 0 or ffrom 0 to 359?)
         else if ((y - y_1) > 180.0) wrap_count -= 1;

         float yw = (y + (360.0 * wrap_count));              //yw is the wrapped angle (can exceed one revolution)

         //Position control
         float e = (r - yw);

         ITerm += (pKi * e);                             //Integral wind up limit
         if (ITerm > 150.0) ITerm = 150.0;
         else if (ITerm < -150.0) ITerm = -150.0;          

         DTerm = pLPFa*DTerm -  pLPFb*pKd*(yw-yw_1);

         float u = (pKp * e) + ITerm + DTerm;

         y_1 = y;  //copy current value of y to previous value (y_1) for next control cycle before PA angle added
         //if(s++%100==0) printf("Controller::run(): wrap_count=%i, y=%f, yw=%f, y_1=%f, yw_1=%f, r=%f, ITerm=%f, DTerm=%f, e=%f, u=%f\n", wrap_count, y, yw, y_1, yw_1, r, ITerm, DTerm, e, u);

         if (u > 0)          //Depending on direction we want to apply torque, add or subtract a phase angle of PA for max effective torque.  PA should be equal to one full step angle: if the excitation angle is the same as the current position, we would not move!  
         {                 //You can experiment with "Phase Advance" by increasing PA when operating at high speeds
            y += PA;          //update phase excitation angle
            if (u > motor.uMax)     // limit control effort
               u = motor.uMax;       //saturation limits max current command
         }
         else
         {
            y -= PA;          //update phase excitation angle
            if (u < -motor.uMax)    // limit control effort
               u = -motor.uMax;      //saturation limits max current command
         }

         float U = abs(u);       //

         //if (abs(e) < 0.1) ledPin_HIGH();    // turn on LED if error is less than 0.1
         //else ledPin_LOW();                  //digitalWrite(ledPin, LOW);


         motor.output(-y, round(U));    // update phase currents

         yw_1 = yw;
      }

      return NULL;
   }

   const Motor& motor;
   const Encoder& encoder;
   const char priority;
   //const uint32_t period;

   char thread_stack[THREAD_STACKSIZE_DEFAULT];
   bool go = false;
   xtimer_ticks32_t last_wakeup;

   long wrap_count = 0;  //keeps track of how many revolutions the motor has gone though (so you can command angles outside of 0-360)
   float y_1 = 0.0;
   float yw_1 = 0.0;
   float ITerm = 0.0;
   float DTerm = 0.0;

   const int spr = 200;                // 200 steps per revolution  -- for 400 step/rev, you should only need to edit this value
   const float aps = 360.0/ spr;       // angle per step
   const float PA = aps;            // Phase advance...aps = 1.8 for 200 steps per rev, 0.9 for 400

   //const float Fs = 6500.0;   //Sample frequency in Hz
   const float Fs = 2000.0;   //Sample frequency in Hz
   const uint32_t period = (uint32_t)(1000000.0/Fs);

   const float pKp = 15.0;      //position mode PID values.  Depending on your motor/load/desired performance, you will need to tune these values.  You can also implement your own control scheme
   const float pKi = 0.2;
   const float pKd = 250.0;//1000.0;
   const float pLPF = 30;       //break frequency in hertz

   const float vKp = 0.001;       //velocity mode PID values.  Depending on your motor/load/desired performance, you will need to tune these values.  You can also implement your own control scheme
   const float vKi = 0.001;
   const float vKd = 0.0;
   const float vLPF = 100.0;       //break frequency in hertz

   const float pLPFa = exp(pLPF*-2.0*3.14159/Fs); // z = e^st pole mapping
   const float pLPFb = (1.0-pLPFa);
   const float vLPFa = exp(vLPF*-2.0*3.14159/Fs); // z = e^st pole mapping
   const float vLPFb = (1.0-vLPFa)* Fs * 0.16666667;
};

#endif
