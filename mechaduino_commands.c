/*
 * Copyright (C) 2019 Florian Seybold
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

#include "mechaduino_commands.h"

#include <saul_reg.h>
#include <as5047d.h>
#include <xtimer.h>

#include "mechaduino_params.h"

#include "mechaduino_state.h"

int mod(int xMod, int mMod) {
  return (xMod % mMod + mMod) % mMod;
}

void output(float theta, int effort) {
  (void)theta;
  (void)effort;
}

void oneStep()
{
  if (!dir) {
    stepNumber += 1;
  }
  else {
    stepNumber -= 1;
  }

  //output(1.8 * stepNumber, 64); //updata 1.8 to aps..., second number is control effort
  output(aps * stepNumber, (int)(0.33 * uMAX));
  xtimer_usleep(10000);
}

int calibrate_cmd_handler(int argc, char **argv)
{
  saul_reg_t* enc_dev = saul_reg_find_type(SAUL_SENSE_ANGLE);

  int encoderReading = 0;     //or float?  not sure if we can average for more res?
  int currentencoderReading = 0;
  int lastencoderReading = 0;
  int avg = 10;               //how many readings to average

  int iStart = 0;     //encoder zero position index
  int jStart = 0;
  int stepNo = 0;
  
  int fullStepReadings[spr];
    
  int fullStep = 0;
  int ticks = 0;
  float lookupAngle = 0.0;
  puts("Beginning calibration routine...\n");

  encoderReading = as5047d_read(&enc_dev);
  bool dir = true;
  oneStep();
  xtimer_usleep(500000);

  if ((as5047d_read(&enc_dev) - encoderReading) < 0)   //check which way motor moves when dir = true
  {
    puts("Wired backwards\n");    // rewiring either phase should fix this.  You may get a false message if you happen to be near the point where the encoder rolls over...
    return;
  }

  int stepNumber = 0;
  while (stepNumber != 0) {       //go to step zero
    if (stepNumber > 0) {
      dir = true;
    }
    else
    {
      dir = false;
    }
    oneStep();
    xtimer_usleep(100000);
  }
  dir = true;
  for (int x = 0; x < spr; x++) {     //step through all full step positions, recording their encoder readings

    encoderReading = 0;
    xtimer_usleep(100000);                         //moving too fast may not give accurate readings.  Motor needs time to settle after each step.
    lastencoderReading = as5047d_read(&enc_dev);
        
    for (int reading = 0; reading < avg; reading++) {  //average multple readings at each step
      currentencoderReading = as5047d_read(&enc_dev);

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
   // puts(fullStepReadings[x], DEC);      //print readings as a sanity check
    printf("%.1f", 100.0*x/spr);
    puts("%\n");
    
    oneStep();
  }
 // puts(" ");
 // puts("ticks:");                        //"ticks" represents the number of encoder counts between successive steps... these should be around 82 for a 1.8 degree stepper
 // puts(" ");
  for (int i = 0; i < spr; i++) {
    ticks = fullStepReadings[mod((i + 1), spr)] - fullStepReadings[mod((i), spr)];
    if (ticks < -15000) {
      ticks += cpr;

    }
    else if (ticks > 15000) {
      ticks -= cpr;
    }
   // puts(ticks);

    if (ticks > 1) {                                    //note starting point with iStart,jStart
      for (int j = 0; j < ticks; j++) {
        stepNo = (mod(fullStepReadings[i] + j, cpr));
        // puts(stepNo);
        if (stepNo == 0) {
          iStart = i;
          jStart = j;
        }

      }
    }

    if (ticks < 1) {                                    //note starting point with iStart,jStart
      for (int j = -ticks; j > 0; j--) {
        stepNo = (mod(fullStepReadings[spr - 1 - i] + j, cpr));
        // puts(stepNo);
        if (stepNo == 0) {
          iStart = i;
          jStart = j;
        }

      }
    }

  }




  puts("\n");                   //The code below generates the lookup table by intepolating between full steps and mapping each encoder count to a calibrated angle
  puts("newLookup:\n");          //The lookup table is too big to store in volatile memory, so we must generate and print it on the fly
  puts("\n");                   // in the future, we hope to be able to print this directly to non-volatile memory


  for (int i = iStart; i < (iStart + spr + 1); i++) {
    ticks = fullStepReadings[mod((i + 1), spr)] - fullStepReadings[mod((i), spr)];

    if (ticks < -15000) {           //check if current interval wraps over encoder's zero positon
      ticks += cpr;
    }
    else if (ticks > 15000) {
      ticks -= cpr;
    }
    //Here we print an interpolated angle corresponding to each encoder count (in order)
    if (ticks > 1) {              //if encoder counts were increasing during cal routine...

      if (i == iStart) { //this is an edge case
        for (int j = jStart; j < ticks; j++) {
          lookupAngle = 0.001 * mod(1000 * ((aps * i) + ((aps * j ) / (float)ticks)), 360000.0);
          printf("%.6f", lookupAngle);
          puts(" , ");
        }
      }

      else if (i == (iStart + spr)) { //this is an edge case
        for (int j = 0; j < jStart; j++) {
          lookupAngle = 0.001 * mod(1000 * ((aps * i) + ((aps * j ) / (float)ticks)), 360000.0);
          printf("%.6f", lookupAngle);
          puts(" , ");
        }
      }
      else {                        //this is the general case
        for (int j = 0; j < ticks; j++) {
          lookupAngle = 0.001 * mod(1000 * ((aps * i) + ((aps * j ) / (float)ticks)), 360000.0);
          printf("%.6f", lookupAngle);
          puts(" , ");
        }
      }



    }

    else if (ticks < 1) {             //similar to above... for case when encoder counts were decreasing during cal routine
      if (i == iStart) {
        for (int j = - ticks; j > (jStart); j--) {
          lookupAngle = 0.001 * mod(1000 * (aps * (i) + (aps * ((ticks + j)) / (float)ticks)), 360000.0);
          printf("%.6f", lookupAngle);
          puts(" , ");
        }
      }
      else if (i == iStart + spr) {
        for (int j = jStart; j > 0; j--) {
          lookupAngle = 0.001 * mod(1000 * (aps * (i) + (aps * ((ticks + j)) / (float)ticks)), 360000.0);
          printf("%.6f", lookupAngle);
          puts(" , ");
        }
      }
      else {
        for (int j = - ticks; j > 0; j--) {
          lookupAngle = 0.001 * mod(1000 * (aps * (i) + (aps * ((ticks + j)) / (float)ticks)), 360000.0);
          printf("%.6f", lookupAngle);
          puts(" , ");
        }
      }

    }


  }
  puts("\n");
}
