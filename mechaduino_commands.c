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
#include <periph/pwm.h>

#include "mechaduino_params.h"

#include "mechaduino_state.h"

#define ENABLE_DEBUG    (0)
#include "debug.h"

int mod(int xMod, int mMod) {
  return (xMod % mMod + mMod) % mMod;
}

void output(float theta, int effort) {
   int angle_1;
   int angle_2;
   int v_coil_A;
   int v_coil_B;

   int sin_coil_A;
   int sin_coil_B;
   int phase_multiplier = 10 * spr / 4;

  //REG_PORT_OUTCLR0 = PORT_PA09; for debugging/timing

  angle_1 = mod((phase_multiplier * theta) , 3600);   //
  angle_2 = mod((phase_multiplier * theta)+900 , 3600);
  
  sin_coil_A  = sin_1[angle_1];

  sin_coil_B = sin_1[angle_2];

  v_coil_A = ((effort * sin_coil_A) / 1024);
  v_coil_B = ((effort * sin_coil_B) / 1024);

/*    // For debugging phase voltages:
     SerialUSB.print(v_coil_A);
     SerialUSB.print(",");
     SerialUSB.println(v_coil_B);
*/
  //analogFastWrite(VREF_1, abs(v_coil_A));
  //analogFastWrite(VREF_2, abs(v_coil_B));
  DEBUG("Set pwm on VREF_1...\n");
  pwm_set(PWM_DEV(1), 1, abs(v_coil_A)); //VREF_1
  DEBUG("Set pwm on VREF_2...\n");
  pwm_set(PWM_DEV(0), 0, abs(v_coil_B)); //VREF_2

  DEBUG("Set pins IN_1 and IN_2...\n");
  if (v_coil_A >= 0)  {
    gpio_set(IN_2);  //REG_PORT_OUTSET0 = PORT_PA21;     //write IN_2 HIGH
    gpio_clear(IN_1);   //REG_PORT_OUTCLR0 = PORT_PA06;     //write IN_1 LOW
  }
  else  {
    gpio_clear(IN_2);   //REG_PORT_OUTCLR0 = PORT_PA21;     //write IN_2 LOW
    gpio_set(IN_1);  //REG_PORT_OUTSET0 = PORT_PA06;     //write IN_1 HIGH
  }

  DEBUG("Set pins IN_3 and IN_4...\n");
  if (v_coil_B >= 0)  {
    gpio_set(IN_4);  //REG_PORT_OUTSET0 = PORT_PA20;     //write IN_4 HIGH
    gpio_clear(IN_3);   //REG_PORT_OUTCLR0 = PORT_PA15;     //write IN_3 LOW
  }
  else  {
    gpio_clear(IN_4);     //REG_PORT_OUTCLR0 = PORT_PA20;     //write IN_4 LOW
    gpio_set(IN_3);    //REG_PORT_OUTSET0 = PORT_PA15;     //write IN_3 HIGH
  }
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
  DEBUG("Call output for one step...\n");
  output(aps * stepNumber, (int)(0.33 * uMAX));
  DEBUG("Sleeping...\n");
  xtimer_usleep(10000);
  DEBUG("Returning from oneStep()...\n");
}

int calibrate_cmd_handler(int argc, char **argv)
{
  int16_t encoderReading = 0;     //or float?  not sure if we can average for more res?
  int16_t currentencoderReading = 0;
  int16_t lastencoderReading = 0;
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

  DEBUG("Checking if wired backwards...\n");
  if ((as5047d_read(&enc_dev) - encoderReading) < 0)   //check which way motor moves when dir = true
  {
    puts("Wired backwards\n");    // rewiring either phase should fix this.  You may get a false message if you happen to be near the point where the encoder rolls over...
    return;
  }

  int stepNumber = 0;
  while (stepNumber != 0) {       //go to step zero
    DEBUG("Running step %i\n", stepNumber);
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
    DEBUG("Running step %i\n", x);

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
    DEBUG("Full step readings: %i\n", fullStepReadings[x]);      //print readings as a sanity check
    printf("%.1f", 100.0*x/spr);
    puts("%\n");
    
    oneStep();
    DEBUG("...one step!\n");
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
