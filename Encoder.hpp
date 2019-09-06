/*
 * Copyright (C) 2019 Florian Seybold
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * stepper.directory for more details.
 */

/**
 * @ingroup     mechaduino_firmware
 *
 * @{
 * @file
 * @brief       Mechaduino encoder
 *
 * @author      Florian Seybold <florian@seybold.space>
 */

#ifndef ENCODER_HPP
#define ENCODER_HPP

#include <xtimer.h>
#include <periph/flashpage.h>

#include "as5047d_params.h"
#include "Stepper.hpp"

class Encoder
{
public:
   Encoder()
   {
      if (as5047d_init(&enc_dev, &as5047d_params[0])) {
         puts("[Init of as5047d failed]");
      }
   }

   int16_t read()
   {
      as5047d_read(&enc_dev);
   }

   /// this is the calibration routine
   void calibrate(Stepper& stepper)
   {
      int encoderReading = 0;     //or float?  not sure if we can average for more res?
      int currentencoderReading = 0;
      int lastencoderReading = 0;
      int avg = 10;               //how many readings to average

      int iStart = 0;     //encoder zero position index
      int jStart = 0;
      int stepNo = 0;

      int fullStepReadings[stepper.motor.spr];

      int fullStep = 0;
      int ticks = 0;
      float lookupAngle = 0.0;
      //SerialUSB.println("Beginning calibration routine...");
      puts("calibrate(): Beginning calibration routine...");

      encoderReading = read();
      stepper.dir = true;
      stepper.step();
      xtimer_usleep(500000);

      if ((read() - encoderReading) < 0)   //check which way motor moves when stepper.dir = true
      {
         puts("calibrate(): Wired backwards");    // rewiring either phase should fix this.  You may get a false message if you happen to be near the point where the encoder rolls over...
         return;
      }

      while (stepper.stepNumber != 0) {       //go to step zero
         if (stepper.stepNumber > 0) {
            stepper.dir = true;
         }
         else
         {
            stepper.dir = false;
         }
         stepper.step();
         xtimer_usleep(100000);
      }
      stepper.dir = true;
      for (int x = 0; x < stepper.motor.spr; x++) {     //step through all full step positions, recording their encoder readings

         encoderReading = 0;
         xtimer_usleep(20000);                           //moving too fast may not give accurate readings.  Motor needs time to settle after each step.
         lastencoderReading = read();

         for (int reading = 0; reading < avg; reading++) {  //average multple readings at each step
            currentencoderReading = read();

            if ((currentencoderReading-lastencoderReading)<(-(cpr/2))){
               currentencoderReading += cpr;
            }
            else if ((currentencoderReading-lastencoderReading)>((cpr/2))){
               currentencoderReading -= cpr;
            }

            encoderReading += currentencoderReading;
            xtimer_usleep(10000);
            lastencoderReading = currentencoderReading;
         }
         encoderReading = encoderReading / avg;
         if (encoderReading>cpr){
            encoderReading-= cpr;
         }
         else if (encoderReading<0){
            encoderReading+= cpr;
         }

         fullStepReadings[x] = encoderReading;
         // SerialUSB.println(fullStepReadings[x], DEC);      //print readings as a sanity check
         /*if (x % 20 == 0)
           {
           SerialUSB.println();
           SerialUSB.print(100*x/stepper.motor.spr);
           SerialUSB.print("% ");
           } else {
           SerialUSB.print('.');
           }*/

         stepper.step();
      }
      //SerialUSB.println();

      // SerialUSB.println(" ");
      // SerialUSB.println("ticks:");                        //"ticks" represents the number of encoder counts between successive steps... these should be around 82 for a 1.8 degree stepper
      // SerialUSB.println(" ");
      for (int i = 0; i < stepper.motor.spr; i++) {
         ticks = fullStepReadings[stepper.motor.mod((i + 1), stepper.motor.spr)] - fullStepReadings[stepper.motor.mod((i), stepper.motor.spr)];
         if (ticks < -15000) {
            ticks += cpr;

         }
         else if (ticks > 15000) {
            ticks -= cpr;
         }
         // SerialUSB.println(ticks);

         if (ticks > 1) {                                    //note starting point with iStart,jStart
            for (int j = 0; j < ticks; j++) {
               stepNo = (stepper.motor.mod(fullStepReadings[i] + j, cpr));
               // SerialUSB.println(stepNo);
               if (stepNo == 0) {
                  iStart = i;
                  jStart = j;
               }

            }
         }

         if (ticks < 1) {                                    //note starting point with iStart,jStart
            for (int j = -ticks; j > 0; j--) {
               stepNo = (stepper.motor.mod(fullStepReadings[stepper.motor.spr - 1 - i] + j, cpr));
               // SerialUSB.println(stepNo);
               if (stepNo == 0) {
                  iStart = i;
                  jStart = j;
               }

            }
         }

      }

      // The code below generates the lookup table by intepolating between
      // full steps and mapping each encoder count to a calibrated angle
      // The lookup table is too big to store in volatile memory,
      // so we must generate and store it into the flash on the fly

      // begin the write to the calibration table
      //const void * page_ptr = (const uint8_t*) lookup;
      //SerialUSB.print("Writing to flash 0x");
      //SerialUSB.print((uintptr_t) page_ptr, HEX);
      //SerialUSB.print(" page size PSZ=");
      //SerialUSB.print(NVMCTRL->PARAM.bit.PSZ);

      for (int i = iStart; i < (iStart + stepper.motor.spr + 1); i++) {
         ticks = fullStepReadings[stepper.motor.mod((i + 1), stepper.motor.spr)] - fullStepReadings[stepper.motor.mod((i), stepper.motor.spr)];

         if (ticks < -15000) {           //check if current interval wrstepper.motor.aps over encoder's zero positon
            ticks += cpr;
         }
         else if (ticks > 15000) {
            ticks -= cpr;
         }

         //Here we print an interpolated angle corresponding to each encoder count (in order)
         if (ticks > 1) {              //if encoder counts were increasing during cal routine...
            if (i == iStart) { //this is an edge case
               for (int j = jStart; j < ticks; j++) {
                  store_lookup(0.001 * stepper.motor.mod(1000 * ((stepper.motor.aps * i) + ((stepper.motor.aps * j ) / float(ticks))), 360000.0));
               }
            }

            else if (i == (iStart + stepper.motor.spr)) { //this is an edge case
               for (int j = 0; j < jStart; j++) {
                  store_lookup(0.001 * stepper.motor.mod(1000 * ((stepper.motor.aps * i) + ((stepper.motor.aps * j ) / float(ticks))), 360000.0));
               }
            }
            else {                        //this is the general case
               for (int j = 0; j < ticks; j++) {
                  store_lookup(0.001 * stepper.motor.mod(1000 * ((stepper.motor.aps * i) + ((stepper.motor.aps * j ) / float(ticks))), 360000.0));
               }
            }
         }

         else if (ticks < 1) {             //similar to above... for case when encoder counts were decreasing during cal routine
            if (i == iStart) {
               for (int j = - ticks; j > (jStart); j--) {
                  store_lookup(0.001 * stepper.motor.mod(1000 * (stepper.motor.aps * (i) + (stepper.motor.aps * ((ticks + j)) / float(ticks))), 360000.0));
               }
            }
            else if (i == iStart + stepper.motor.spr) {
               for (int j = jStart; j > 0; j--) {
                  store_lookup(0.001 * stepper.motor.mod(1000 * (stepper.motor.aps * (i) + (stepper.motor.aps * ((ticks + j)) / float(ticks))), 360000.0));
               }
            }
            else {
               for (int j = - ticks; j > 0; j--) {
                  store_lookup(0.001 * stepper.motor.mod(1000 * (stepper.motor.aps * (i) + (stepper.motor.aps * ((ticks + j)) / float(ticks))), 360000.0));
               }
            }

            puts("calibrate(): Publishing lookup.");
         }


      }

      if (page_count != 0)
         write_page();

      //SerialUSB.println(" ");
      //SerialUSB.println(" ");
      //SerialUSB.println("Calibration complete!");
      //SerialUSB.println("The calibration table has been written to non-volatile Flash memory!");
      //SerialUSB.println(" ");
      //SerialUSB.println(" ");
   }

private:
   void write_page()
   {
      /*if (0 == (0xFFF & (uintptr_t) page_ptr))
        {
        SerialUSB.println();
        SerialUSB.print("0x");
        SerialUSB.print((uintptr_t) page_ptr, HEX);
        } else {
        SerialUSB.print(".");
        }*/

      //flash.erase((const void*) page_ptr, sizeof(page));
      //flash.write((const void*) page_ptr, (const void *) page, sizeof(page));
      flashpage_write(page_number, page);
   }

   void store_lookup(const float& lookupAngle)
   {
      page[page_count++] = lookupAngle;
      if(page_count != floats_per_page)
         return;

      // we've filled an entire page, write it to the flash
      write_page();

      // reset our counters and increment our flash page
      page_number += 1;
      page_count = 0;
      memset(page, 0, sizeof(page));
   }


   as5047d_t enc_dev;

   unsigned page_count = 0;
   unsigned page_number = flashpage_page(&lookup);

   static const unsigned page_size = FLASHPAGE_SIZE; // actual size is 64?
   static const unsigned floats_per_page = page_size / sizeof(float);
   float page[floats_per_page];

   const int cpr = 16384;                    // counts per rev

   float __attribute__((__aligned__(256))) lookup[16384];
};

#endif
