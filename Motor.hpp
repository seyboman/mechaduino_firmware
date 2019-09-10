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

#ifndef MOTOR_HPP
#define MOTOR_HPP

#include "arduino_pinmap.h"

//Defines for pins:
#define IN_4  ARDUINO_PIN_6
#define IN_3  ARDUINO_PIN_5
#define VREF_2 ARDUINO_PIN_4
#define VREF_1 ARDUINO_PIN_9
#define IN_2  ARDUINO_PIN_7
#define IN_1  ARDUINO_PIN_8
#define ledPin  ARDUINO_PIN_13
#define chipSelectPin ARDUINO_PIN_A2 //output to chip select

#include <periph/pwm.h>

#define ENABLE_DEBUG    (0)
#include "debug.h"


class Motor
{
public:
   Motor()
   {
      DEBUG("Initializing motor...\n");

      gpio_init(IN_4, GPIO_OUT);
      gpio_init(IN_3, GPIO_OUT);
      gpio_init(IN_2, GPIO_OUT);
      gpio_init(IN_1, GPIO_OUT);

      pwm_init(PWM_DEV(0), PWM_LEFT, 187000, 255);
      pwm_set(PWM_DEV(0), 0, 0.33 * uMax); //VREF_2
      pwm_init(PWM_DEV(1), PWM_LEFT, 187000, 255);
      pwm_set(PWM_DEV(1), 0, 0.33 * uMax); //VREF_1

      gpio_set(IN_4);
      gpio_clear(IN_3);
      gpio_set(IN_2);
      gpio_clear(IN_1);
   }

   int mod(int xMod, int mMod) const {
      return (xMod % mMod + mMod) % mMod;
   }

   void output(const float& theta, const int& effort) const
   {
      const int phase_multiplier = 10 * spr / 4;

      int angle_1 = mod((phase_multiplier * theta) , 3600);   //
      int angle_2 = mod((phase_multiplier * theta)+900 , 3600);

      int sin_coil_A  = sin_1[angle_1];
      int sin_coil_B = sin_1[angle_2];

      int v_coil_A = ((effort * sin_coil_A) / 1024);
      int v_coil_B = ((effort * sin_coil_B) / 1024);
      //DEBUG("Compute angle_1=%i, angle_2=%i, sin_coil_A=%i, sin_coil_B=%i, v_coil_A=%i, v_coil_B=%i\n", angle_1, angle_2, sin_coil_A, sin_coil_B, v_coil_A, v_coil_B);

      pwm_set(PWM_DEV(1), 0, abs(v_coil_A)); //VREF_1
      pwm_set(PWM_DEV(0), 0, abs(v_coil_B)); //VREF_2

      if (v_coil_A >= 0)  {
         gpio_set(IN_2);  //REG_PORT_OUTSET0 = PORT_PA21;     //write IN_2 HIGH
         gpio_clear(IN_1);   //REG_PORT_OUTCLR0 = PORT_PA06;     //write IN_1 LOW
      }
      else  {
         gpio_clear(IN_2);   //REG_PORT_OUTCLR0 = PORT_PA21;     //write IN_2 LOW
         gpio_set(IN_1);  //REG_PORT_OUTSET0 = PORT_PA06;     //write IN_1 HIGH
      }

      if (v_coil_B >= 0)  {
         gpio_set(IN_4);  //REG_PORT_OUTSET0 = PORT_PA20;     //write IN_4 HIGH
         gpio_clear(IN_3);   //REG_PORT_OUTCLR0 = PORT_PA15;     //write IN_3 LOW
      }
      else  {
         gpio_clear(IN_4);     //REG_PORT_OUTCLR0 = PORT_PA20;     //write IN_4 LOW
         gpio_set(IN_3);    //REG_PORT_OUTSET0 = PORT_PA15;     //write IN_3 HIGH
      }
   }

   const int spr = 200;                // 200 steps per revolution  -- for 400 step/rev, you should only need to edit this value
   const int uMax = (int)((255.0/3.3)*(iMax*10.0*rSense));   // 255 for 8-bit pwm, 1023 for 10 bit, must also edit analogFastWrite
   const float aps = 360.0/ spr;       // angle per step

private:
   const float iMax = 1.0;             // Be careful adjusting this.  While the A4954 driver is rated for 2.0 Amp peak currents, it cannot handle these currents continuously.  Depending on how you operate the Mechaduino, you may be able to safely raise this value...please refer to the A4954 datasheet for more info
   const float rSense = 0.150;

   static const int sin_1[];
};

const int Motor::sin_1[] = {
    +0,    +2,    +4,    +5,    +7,    +9,   +11,   +13,   +14,   +16,
   +18,   +20,   +21,   +23,   +25,   +27,   +29,   +30,   +32,   +34,
   +36,   +38,   +39,   +41,   +43,   +45,   +46,   +48,   +50,   +52,
   +54,   +55,   +57,   +59,   +61,   +63,   +64,   +66,   +68,   +70,
   +71,   +73,   +75,   +77,   +79,   +80,   +82,   +84,   +86,   +87,
   +89,   +91,   +93,   +95,   +96,   +98,  +100,  +102,  +103,  +105,
  +107,  +109,  +111,  +112,  +114,  +116,  +118,  +119,  +121,  +123,
  +125,  +127,  +128,  +130,  +132,  +134,  +135,  +137,  +139,  +141,
  +143,  +144,  +146,  +148,  +150,  +151,  +153,  +155,  +157,  +158,
  +160,  +162,  +164,  +165,  +167,  +169,  +171,  +173,  +174,  +176,
  +178,  +180,  +181,  +183,  +185,  +187,  +188,  +190,  +192,  +194,
  +195,  +197,  +199,  +201,  +202,  +204,  +206,  +208,  +209,  +211,
  +213,  +215,  +216,  +218,  +220,  +222,  +223,  +225,  +227,  +229,
  +230,  +232,  +234,  +236,  +237,  +239,  +241,  +243,  +244,  +246,
  +248,  +249,  +251,  +253,  +255,  +256,  +258,  +260,  +262,  +263,
  +265,  +267,  +268,  +270,  +272,  +274,  +275,  +277,  +279,  +281,
  +282,  +284,  +286,  +287,  +289,  +291,  +293,  +294,  +296,  +298,
  +299,  +301,  +303,  +305,  +306,  +308,  +310,  +311,  +313,  +315,
  +316,  +318,  +320,  +322,  +323,  +325,  +327,  +328,  +330,  +332,
  +333,  +335,  +337,  +338,  +340,  +342,  +343,  +345,  +347,  +349,
  +350,  +352,  +354,  +355,  +357,  +359,  +360,  +362,  +364,  +365,
  +367,  +369,  +370,  +372,  +374,  +375,  +377,  +379,  +380,  +382,
  +384,  +385,  +387,  +389,  +390,  +392,  +394,  +395,  +397,  +398,
  +400,  +402,  +403,  +405,  +407,  +408,  +410,  +412,  +413,  +415,
  +416,  +418,  +420,  +421,  +423,  +425,  +426,  +428,  +430,  +431,
  +433,  +434,  +436,  +438,  +439,  +441,  +442,  +444,  +446,  +447,
  +449,  +450,  +452,  +454,  +455,  +457,  +458,  +460,  +462,  +463,
  +465,  +466,  +468,  +470,  +471,  +473,  +474,  +476,  +478,  +479,
  +481,  +482,  +484,  +485,  +487,  +489,  +490,  +492,  +493,  +495,
  +496,  +498,  +500,  +501,  +503,  +504,  +506,  +507,  +509,  +510,
  +512,  +514,  +515,  +517,  +518,  +520,  +521,  +523,  +524,  +526,
  +527,  +529,  +530,  +532,  +533,  +535,  +537,  +538,  +540,  +541,
  +543,  +544,  +546,  +547,  +549,  +550,  +552,  +553,  +555,  +556,
  +558,  +559,  +561,  +562,  +564,  +565,  +567,  +568,  +570,  +571,
  +573,  +574,  +576,  +577,  +579,  +580,  +581,  +583,  +584,  +586,
  +587,  +589,  +590,  +592,  +593,  +595,  +596,  +598,  +599,  +600,
  +602,  +603,  +605,  +606,  +608,  +609,  +611,  +612,  +613,  +615,
  +616,  +618,  +619,  +621,  +622,  +623,  +625,  +626,  +628,  +629,
  +630,  +632,  +633,  +635,  +636,  +637,  +639,  +640,  +642,  +643,
  +644,  +646,  +647,  +649,  +650,  +651,  +653,  +654,  +655,  +657,
  +658,  +660,  +661,  +662,  +664,  +665,  +666,  +668,  +669,  +670,
  +672,  +673,  +674,  +676,  +677,  +679,  +680,  +681,  +683,  +684,
  +685,  +687,  +688,  +689,  +690,  +692,  +693,  +694,  +696,  +697,
  +698,  +700,  +701,  +702,  +704,  +705,  +706,  +707,  +709,  +710,
  +711,  +713,  +714,  +715,  +716,  +718,  +719,  +720,  +722,  +723,
  +724,  +725,  +727,  +728,  +729,  +730,  +732,  +733,  +734,  +735,
  +737,  +738,  +739,  +740,  +742,  +743,  +744,  +745,  +746,  +748,
  +749,  +750,  +751,  +753,  +754,  +755,  +756,  +757,  +759,  +760,
  +761,  +762,  +763,  +765,  +766,  +767,  +768,  +769,  +770,  +772,
  +773,  +774,  +775,  +776,  +777,  +779,  +780,  +781,  +782,  +783,
  +784,  +786,  +787,  +788,  +789,  +790,  +791,  +792,  +794,  +795,
  +796,  +797,  +798,  +799,  +800,  +801,  +802,  +804,  +805,  +806,
  +807,  +808,  +809,  +810,  +811,  +812,  +813,  +815,  +816,  +817,
  +818,  +819,  +820,  +821,  +822,  +823,  +824,  +825,  +826,  +827,
  +828,  +829,  +831,  +832,  +833,  +834,  +835,  +836,  +837,  +838,
  +839,  +840,  +841,  +842,  +843,  +844,  +845,  +846,  +847,  +848,
  +849,  +850,  +851,  +852,  +853,  +854,  +855,  +856,  +857,  +858,
  +859,  +860,  +861,  +862,  +863,  +864,  +865,  +866,  +866,  +867,
  +868,  +869,  +870,  +871,  +872,  +873,  +874,  +875,  +876,  +877,
  +878,  +879,  +880,  +880,  +881,  +882,  +883,  +884,  +885,  +886,
  +887,  +888,  +889,  +889,  +890,  +891,  +892,  +893,  +894,  +895,
  +896,  +896,  +897,  +898,  +899,  +900,  +901,  +902,  +902,  +903,
  +904,  +905,  +906,  +907,  +907,  +908,  +909,  +910,  +911,  +912,
  +912,  +913,  +914,  +915,  +916,  +916,  +917,  +918,  +919,  +920,
  +920,  +921,  +922,  +923,  +923,  +924,  +925,  +926,  +927,  +927,
  +928,  +929,  +930,  +930,  +931,  +932,  +933,  +933,  +934,  +935,
  +935,  +936,  +937,  +938,  +938,  +939,  +940,  +940,  +941,  +942,
  +943,  +943,  +944,  +945,  +945,  +946,  +947,  +947,  +948,  +949,
  +949,  +950,  +951,  +951,  +952,  +953,  +953,  +954,  +955,  +955,
  +956,  +957,  +957,  +958,  +959,  +959,  +960,  +960,  +961,  +962,
  +962,  +963,  +963,  +964,  +965,  +965,  +966,  +966,  +967,  +968,
  +968,  +969,  +969,  +970,  +971,  +971,  +972,  +972,  +973,  +973,
  +974,  +974,  +975,  +976,  +976,  +977,  +977,  +978,  +978,  +979,
  +979,  +980,  +980,  +981,  +981,  +982,  +982,  +983,  +983,  +984,
  +984,  +985,  +985,  +986,  +986,  +987,  +987,  +988,  +988,  +989,
  +989,  +990,  +990,  +990,  +991,  +991,  +992,  +992,  +993,  +993,
  +994,  +994,  +994,  +995,  +995,  +996,  +996,  +997,  +997,  +997,
  +998,  +998,  +999,  +999,  +999, +1000, +1000, +1000, +1001, +1001,
 +1002, +1002, +1002, +1003, +1003, +1003, +1004, +1004, +1004, +1005,
 +1005, +1006, +1006, +1006, +1007, +1007, +1007, +1007, +1008, +1008,
 +1008, +1009, +1009, +1009, +1010, +1010, +1010, +1011, +1011, +1011,
 +1011, +1012, +1012, +1012, +1012, +1013, +1013, +1013, +1014, +1014,
 +1014, +1014, +1015, +1015, +1015, +1015, +1015, +1016, +1016, +1016,
 +1016, +1017, +1017, +1017, +1017, +1017, +1018, +1018, +1018, +1018,
 +1018, +1019, +1019, +1019, +1019, +1019, +1019, +1020, +1020, +1020,
 +1020, +1020, +1020, +1021, +1021, +1021, +1021, +1021, +1021, +1021,
 +1022, +1022, +1022, +1022, +1022, +1022, +1022, +1022, +1022, +1022,
 +1023, +1023, +1023, +1023, +1023, +1023, +1023, +1023, +1023, +1023,
 +1023, +1023, +1023, +1024, +1024, +1024, +1024, +1024, +1024, +1024,
 +1024, +1024, +1024, +1024, +1024, +1024, +1024, +1024, +1024, +1024,
 +1024, +1024, +1024, +1024, +1024, +1024, +1024, +1024, +1024, +1024,
 +1024, +1024, +1024, +1024, +1024, +1024, +1024, +1024, +1023, +1023,
 +1023, +1023, +1023, +1023, +1023, +1023, +1023, +1023, +1023, +1023,
 +1023, +1023, +1022, +1022, +1022, +1022, +1022, +1022, +1022, +1022,
 +1022, +1021, +1021, +1021, +1021, +1021, +1021, +1021, +1020, +1020,
 +1020, +1020, +1020, +1020, +1019, +1019, +1019, +1019, +1019, +1019,
 +1018, +1018, +1018, +1018, +1018, +1017, +1017, +1017, +1017, +1017,
 +1016, +1016, +1016, +1016, +1015, +1015, +1015, +1015, +1015, +1014,
 +1014, +1014, +1014, +1013, +1013, +1013, +1012, +1012, +1012, +1012,
 +1011, +1011, +1011, +1011, +1010, +1010, +1010, +1009, +1009, +1009,
 +1008, +1008, +1008, +1008, +1007, +1007, +1007, +1006, +1006, +1006,
 +1005, +1005, +1005, +1004, +1004, +1003, +1003, +1003, +1002, +1002,
 +1002, +1001, +1001, +1001, +1000, +1000,  +999,  +999,  +999,  +998,
  +998,  +997,  +997,  +997,  +996,  +996,  +995,  +995,  +994,  +994,
  +994,  +993,  +993,  +992,  +992,  +991,  +991,  +990,  +990,  +990,
  +989,  +989,  +988,  +988,  +987,  +987,  +986,  +986,  +985,  +985,
  +984,  +984,  +983,  +983,  +982,  +982,  +981,  +981,  +980,  +980,
  +979,  +979,  +978,  +978,  +977,  +977,  +976,  +976,  +975,  +974,
  +974,  +973,  +973,  +972,  +972,  +971,  +971,  +970,  +969,  +969,
  +968,  +968,  +967,  +966,  +966,  +965,  +965,  +964,  +963,  +963,
  +962,  +962,  +961,  +960,  +960,  +959,  +959,  +958,  +957,  +957,
  +956,  +955,  +955,  +954,  +953,  +953,  +952,  +951,  +951,  +950,
  +949,  +949,  +948,  +947,  +947,  +946,  +945,  +945,  +944,  +943,
  +943,  +942,  +941,  +941,  +940,  +939,  +938,  +938,  +937,  +936,
  +935,  +935,  +934,  +933,  +933,  +932,  +931,  +930,  +930,  +929,
  +928,  +927,  +927,  +926,  +925,  +924,  +924,  +923,  +922,  +921,
  +920,  +920,  +919,  +918,  +917,  +916,  +916,  +915,  +914,  +913,
  +912,  +912,  +911,  +910,  +909,  +908,  +908,  +907,  +906,  +905,
  +904,  +903,  +902,  +902,  +901,  +900,  +899,  +898,  +897,  +897,
  +896,  +895,  +894,  +893,  +892,  +891,  +890,  +890,  +889,  +888,
  +887,  +886,  +885,  +884,  +883,  +882,  +881,  +881,  +880,  +879,
  +878,  +877,  +876,  +875,  +874,  +873,  +872,  +871,  +870,  +869,
  +868,  +867,  +867,  +866,  +865,  +864,  +863,  +862,  +861,  +860,
  +859,  +858,  +857,  +856,  +855,  +854,  +853,  +852,  +851,  +850,
  +849,  +848,  +847,  +846,  +845,  +844,  +843,  +842,  +841,  +840,
  +839,  +838,  +837,  +836,  +835,  +834,  +833,  +832,  +831,  +830,
  +828,  +827,  +826,  +825,  +824,  +823,  +822,  +821,  +820,  +819,
  +818,  +817,  +816,  +815,  +814,  +812,  +811,  +810,  +809,  +808,
  +807,  +806,  +805,  +804,  +803,  +801,  +800,  +799,  +798,  +797,
  +796,  +795,  +794,  +792,  +791,  +790,  +789,  +788,  +787,  +786,
  +784,  +783,  +782,  +781,  +780,  +779,  +778,  +776,  +775,  +774,
  +773,  +772,  +771,  +769,  +768,  +767,  +766,  +765,  +763,  +762,
  +761,  +760,  +759,  +757,  +756,  +755,  +754,  +753,  +751,  +750,
  +749,  +748,  +747,  +745,  +744,  +743,  +742,  +740,  +739,  +738,
  +737,  +735,  +734,  +733,  +732,  +730,  +729,  +728,  +727,  +725,
  +724,  +723,  +722,  +720,  +719,  +718,  +717,  +715,  +714,  +713,
  +711,  +710,  +709,  +708,  +706,  +705,  +704,  +702,  +701,  +700,
  +698,  +697,  +696,  +694,  +693,  +692,  +691,  +689,  +688,  +687,
  +685,  +684,  +683,  +681,  +680,  +679,  +677,  +676,  +675,  +673,
  +672,  +671,  +669,  +668,  +666,  +665,  +664,  +662,  +661,  +660,
  +658,  +657,  +656,  +654,  +653,  +651,  +650,  +649,  +647,  +646,
  +644,  +643,  +642,  +640,  +639,  +638,  +636,  +635,  +633,  +632,
  +630,  +629,  +628,  +626,  +625,  +623,  +622,  +621,  +619,  +618,
  +616,  +615,  +613,  +612,  +611,  +609,  +608,  +606,  +605,  +603,
  +602,  +601,  +599,  +598,  +596,  +595,  +593,  +592,  +590,  +589,
  +587,  +586,  +584,  +583,  +582,  +580,  +579,  +577,  +576,  +574,
  +573,  +571,  +570,  +568,  +567,  +565,  +564,  +562,  +561,  +559,
  +558,  +556,  +555,  +553,  +552,  +550,  +549,  +547,  +546,  +544,
  +543,  +541,  +540,  +538,  +537,  +535,  +534,  +532,  +531,  +529,
  +527,  +526,  +524,  +523,  +521,  +520,  +518,  +517,  +515,  +514,
  +512,  +511,  +509,  +507,  +506,  +504,  +503,  +501,  +500,  +498,
  +497,  +495,  +493,  +492,  +490,  +489,  +487,  +486,  +484,  +482,
  +481,  +479,  +478,  +476,  +474,  +473,  +471,  +470,  +468,  +467,
  +465,  +463,  +462,  +460,  +459,  +457,  +455,  +454,  +452,  +451,
  +449,  +447,  +446,  +444,  +443,  +441,  +439,  +438,  +436,  +434,
  +433,  +431,  +430,  +428,  +426,  +425,  +423,  +421,  +420,  +418,
  +417,  +415,  +413,  +412,  +410,  +408,  +407,  +405,  +403,  +402,
  +400,  +399,  +397,  +395,  +394,  +392,  +390,  +389,  +387,  +385,
  +384,  +382,  +380,  +379,  +377,  +375,  +374,  +372,  +370,  +369,
  +367,  +365,  +364,  +362,  +360,  +359,  +357,  +355,  +354,  +352,
  +350,  +349,  +347,  +345,  +344,  +342,  +340,  +339,  +337,  +335,
  +333,  +332,  +330,  +328,  +327,  +325,  +323,  +322,  +320,  +318,
  +317,  +315,  +313,  +311,  +310,  +308,  +306,  +305,  +303,  +301,
  +299,  +298,  +296,  +294,  +293,  +291,  +289,  +287,  +286,  +284,
  +282,  +281,  +279,  +277,  +275,  +274,  +272,  +270,  +269,  +267,
  +265,  +263,  +262,  +260,  +258,  +256,  +255,  +253,  +251,  +250,
  +248,  +246,  +244,  +243,  +241,  +239,  +237,  +236,  +234,  +232,
  +230,  +229,  +227,  +225,  +223,  +222,  +220,  +218,  +216,  +215,
  +213,  +211,  +209,  +208,  +206,  +204,  +202,  +201,  +199,  +197,
  +195,  +194,  +192,  +190,  +188,  +187,  +185,  +183,  +181,  +180,
  +178,  +176,  +174,  +173,  +171,  +169,  +167,  +166,  +164,  +162,
  +160,  +159,  +157,  +155,  +153,  +151,  +150,  +148,  +146,  +144,
  +143,  +141,  +139,  +137,  +136,  +134,  +132,  +130,  +128,  +127,
  +125,  +123,  +121,  +120,  +118,  +116,  +114,  +112,  +111,  +109,
  +107,  +105,  +104,  +102,  +100,   +98,   +96,   +95,   +93,   +91,
   +89,   +88,   +86,   +84,   +82,   +80,   +79,   +77,   +75,   +73,
   +72,   +70,   +68,   +66,   +64,   +63,   +61,   +59,   +57,   +55,
   +54,   +52,   +50,   +48,   +47,   +45,   +43,   +41,   +39,   +38,
   +36,   +34,   +32,   +30,   +29,   +27,   +25,   +23,   +22,   +20,
   +18,   +16,   +14,   +13,   +11,    +9,    +7,    +5,    +4,    +2,
    +0,    -2,    -3,    -5,    -7,    -9,   -11,   -12,   -14,   -16,
   -18,   -20,   -21,   -23,   -25,   -27,   -28,   -30,   -32,   -34,
   -36,   -37,   -39,   -41,   -43,   -45,   -46,   -48,   -50,   -52,
   -53,   -55,   -57,   -59,   -61,   -62,   -64,   -66,   -68,   -70,
   -71,   -73,   -75,   -77,   -78,   -80,   -82,   -84,   -86,   -87,
   -89,   -91,   -93,   -94,   -96,   -98,  -100,  -102,  -103,  -105,
  -107,  -109,  -110,  -112,  -114,  -116,  -118,  -119,  -121,  -123,
  -125,  -126,  -128,  -130,  -132,  -134,  -135,  -137,  -139,  -141,
  -142,  -144,  -146,  -148,  -149,  -151,  -153,  -155,  -157,  -158,
  -160,  -162,  -164,  -165,  -167,  -169,  -171,  -172,  -174,  -176,
  -178,  -179,  -181,  -183,  -185,  -187,  -188,  -190,  -192,  -194,
  -195,  -197,  -199,  -201,  -202,  -204,  -206,  -208,  -209,  -211,
  -213,  -215,  -216,  -218,  -220,  -222,  -223,  -225,  -227,  -229,
  -230,  -232,  -234,  -235,  -237,  -239,  -241,  -242,  -244,  -246,
  -248,  -249,  -251,  -253,  -255,  -256,  -258,  -260,  -261,  -263,
  -265,  -267,  -268,  -270,  -272,  -274,  -275,  -277,  -279,  -280,
  -282,  -284,  -286,  -287,  -289,  -291,  -292,  -294,  -296,  -298,
  -299,  -301,  -303,  -304,  -306,  -308,  -310,  -311,  -313,  -315,
  -316,  -318,  -320,  -321,  -323,  -325,  -327,  -328,  -330,  -332,
  -333,  -335,  -337,  -338,  -340,  -342,  -343,  -345,  -347,  -348,
  -350,  -352,  -353,  -355,  -357,  -359,  -360,  -362,  -364,  -365,
  -367,  -369,  -370,  -372,  -374,  -375,  -377,  -379,  -380,  -382,
  -383,  -385,  -387,  -388,  -390,  -392,  -393,  -395,  -397,  -398,
  -400,  -402,  -403,  -405,  -407,  -408,  -410,  -411,  -413,  -415,
  -416,  -418,  -420,  -421,  -423,  -425,  -426,  -428,  -429,  -431,
  -433,  -434,  -436,  -438,  -439,  -441,  -442,  -444,  -446,  -447,
  -449,  -450,  -452,  -454,  -455,  -457,  -458,  -460,  -462,  -463,
  -465,  -466,  -468,  -470,  -471,  -473,  -474,  -476,  -477,  -479,
  -481,  -482,  -484,  -485,  -487,  -489,  -490,  -492,  -493,  -495,
  -496,  -498,  -499,  -501,  -503,  -504,  -506,  -507,  -509,  -510,
  -512,  -513,  -515,  -517,  -518,  -520,  -521,  -523,  -524,  -526,
  -527,  -529,  -530,  -532,  -533,  -535,  -536,  -538,  -540,  -541,
  -543,  -544,  -546,  -547,  -549,  -550,  -552,  -553,  -555,  -556,
  -558,  -559,  -561,  -562,  -564,  -565,  -567,  -568,  -570,  -571,
  -573,  -574,  -575,  -577,  -578,  -580,  -581,  -583,  -584,  -586,
  -587,  -589,  -590,  -592,  -593,  -595,  -596,  -597,  -599,  -600,
  -602,  -603,  -605,  -606,  -608,  -609,  -610,  -612,  -613,  -615,
  -616,  -618,  -619,  -620,  -622,  -623,  -625,  -626,  -628,  -629,
  -630,  -632,  -633,  -635,  -636,  -637,  -639,  -640,  -642,  -643,
  -644,  -646,  -647,  -648,  -650,  -651,  -653,  -654,  -655,  -657,
  -658,  -659,  -661,  -662,  -664,  -665,  -666,  -668,  -669,  -670,
  -672,  -673,  -674,  -676,  -677,  -678,  -680,  -681,  -682,  -684,
  -685,  -686,  -688,  -689,  -690,  -692,  -693,  -694,  -696,  -697,
  -698,  -700,  -701,  -702,  -703,  -705,  -706,  -707,  -709,  -710,
  -711,  -713,  -714,  -715,  -716,  -718,  -719,  -720,  -721,  -723,
  -724,  -725,  -727,  -728,  -729,  -730,  -732,  -733,  -734,  -735,
  -737,  -738,  -739,  -740,  -741,  -743,  -744,  -745,  -746,  -748,
  -749,  -750,  -751,  -752,  -754,  -755,  -756,  -757,  -759,  -760,
  -761,  -762,  -763,  -764,  -766,  -767,  -768,  -769,  -770,  -772,
  -773,  -774,  -775,  -776,  -777,  -779,  -780,  -781,  -782,  -783,
  -784,  -785,  -787,  -788,  -789,  -790,  -791,  -792,  -793,  -795,
  -796,  -797,  -798,  -799,  -800,  -801,  -802,  -804,  -805,  -806,
  -807,  -808,  -809,  -810,  -811,  -812,  -813,  -814,  -816,  -817,
  -818,  -819,  -820,  -821,  -822,  -823,  -824,  -825,  -826,  -827,
  -828,  -829,  -830,  -832,  -833,  -834,  -835,  -836,  -837,  -838,
  -839,  -840,  -841,  -842,  -843,  -844,  -845,  -846,  -847,  -848,
  -849,  -850,  -851,  -852,  -853,  -854,  -855,  -856,  -857,  -858,
  -859,  -860,  -861,  -862,  -863,  -864,  -865,  -865,  -866,  -867,
  -868,  -869,  -870,  -871,  -872,  -873,  -874,  -875,  -876,  -877,
  -878,  -879,  -880,  -880,  -881,  -882,  -883,  -884,  -885,  -886,
  -887,  -888,  -889,  -889,  -890,  -891,  -892,  -893,  -894,  -895,
  -896,  -896,  -897,  -898,  -899,  -900,  -901,  -902,  -902,  -903,
  -904,  -905,  -906,  -907,  -907,  -908,  -909,  -910,  -911,  -912,
  -912,  -913,  -914,  -915,  -916,  -916,  -917,  -918,  -919,  -920,
  -920,  -921,  -922,  -923,  -923,  -924,  -925,  -926,  -926,  -927,
  -928,  -929,  -930,  -930,  -931,  -932,  -932,  -933,  -934,  -935,
  -935,  -936,  -937,  -938,  -938,  -939,  -940,  -940,  -941,  -942,
  -943,  -943,  -944,  -945,  -945,  -946,  -947,  -947,  -948,  -949,
  -949,  -950,  -951,  -951,  -952,  -953,  -953,  -954,  -955,  -955,
  -956,  -957,  -957,  -958,  -958,  -959,  -960,  -960,  -961,  -962,
  -962,  -963,  -963,  -964,  -965,  -965,  -966,  -966,  -967,  -968,
  -968,  -969,  -969,  -970,  -970,  -971,  -972,  -972,  -973,  -973,
  -974,  -974,  -975,  -975,  -976,  -977,  -977,  -978,  -978,  -979,
  -979,  -980,  -980,  -981,  -981,  -982,  -982,  -983,  -983,  -984,
  -984,  -985,  -985,  -986,  -986,  -987,  -987,  -988,  -988,  -989,
  -989,  -990,  -990,  -990,  -991,  -991,  -992,  -992,  -993,  -993,
  -994,  -994,  -994,  -995,  -995,  -996,  -996,  -997,  -997,  -997,
  -998,  -998,  -999,  -999,  -999, -1000, -1000, -1000, -1001, -1001,
 -1002, -1002, -1002, -1003, -1003, -1003, -1004, -1004, -1004, -1005,
 -1005, -1005, -1006, -1006, -1007, -1007, -1007, -1007, -1008, -1008,
 -1008, -1009, -1009, -1009, -1010, -1010, -1010, -1011, -1011, -1011,
 -1011, -1012, -1012, -1012, -1012, -1013, -1013, -1013, -1014, -1014,
 -1014, -1014, -1015, -1015, -1015, -1015, -1015, -1016, -1016, -1016,
 -1016, -1017, -1017, -1017, -1017, -1017, -1018, -1018, -1018, -1018,
 -1018, -1019, -1019, -1019, -1019, -1019, -1019, -1020, -1020, -1020,
 -1020, -1020, -1020, -1021, -1021, -1021, -1021, -1021, -1021, -1021,
 -1021, -1022, -1022, -1022, -1022, -1022, -1022, -1022, -1022, -1022,
 -1023, -1023, -1023, -1023, -1023, -1023, -1023, -1023, -1023, -1023,
 -1023, -1023, -1023, -1024, -1024, -1024, -1024, -1024, -1024, -1024,
 -1024, -1024, -1024, -1024, -1024, -1024, -1024, -1024, -1024, -1024,
 -1024, -1024, -1024, -1024, -1024, -1024, -1024, -1024, -1024, -1024,
 -1024, -1024, -1024, -1024, -1024, -1024, -1024, -1024, -1023, -1023,
 -1023, -1023, -1023, -1023, -1023, -1023, -1023, -1023, -1023, -1023,
 -1023, -1023, -1022, -1022, -1022, -1022, -1022, -1022, -1022, -1022,
 -1022, -1021, -1021, -1021, -1021, -1021, -1021, -1021, -1020, -1020,
 -1020, -1020, -1020, -1020, -1019, -1019, -1019, -1019, -1019, -1019,
 -1018, -1018, -1018, -1018, -1018, -1017, -1017, -1017, -1017, -1017,
 -1016, -1016, -1016, -1016, -1015, -1015, -1015, -1015, -1015, -1014,
 -1014, -1014, -1014, -1013, -1013, -1013, -1013, -1012, -1012, -1012,
 -1011, -1011, -1011, -1011, -1010, -1010, -1010, -1009, -1009, -1009,
 -1008, -1008, -1008, -1008, -1007, -1007, -1007, -1006, -1006, -1006,
 -1005, -1005, -1005, -1004, -1004, -1003, -1003, -1003, -1002, -1002,
 -1002, -1001, -1001, -1001, -1000, -1000,  -999,  -999,  -999,  -998,
  -998,  -997,  -997,  -997,  -996,  -996,  -995,  -995,  -994,  -994,
  -994,  -993,  -993,  -992,  -992,  -991,  -991,  -991,  -990,  -990,
  -989,  -989,  -988,  -988,  -987,  -987,  -986,  -986,  -985,  -985,
  -984,  -984,  -983,  -983,  -982,  -982,  -981,  -981,  -980,  -980,
  -979,  -979,  -978,  -978,  -977,  -977,  -976,  -976,  -975,  -974,
  -974,  -973,  -973,  -972,  -972,  -971,  -971,  -970,  -969,  -969,
  -968,  -968,  -967,  -967,  -966,  -965,  -965,  -964,  -964,  -963,
  -962,  -962,  -961,  -960,  -960,  -959,  -959,  -958,  -957,  -957,
  -956,  -955,  -955,  -954,  -953,  -953,  -952,  -951,  -951,  -950,
  -949,  -949,  -948,  -947,  -947,  -946,  -945,  -945,  -944,  -943,
  -943,  -942,  -941,  -941,  -940,  -939,  -938,  -938,  -937,  -936,
  -936,  -935,  -934,  -933,  -933,  -932,  -931,  -930,  -930,  -929,
  -928,  -927,  -927,  -926,  -925,  -924,  -924,  -923,  -922,  -921,
  -920,  -920,  -919,  -918,  -917,  -916,  -916,  -915,  -914,  -913,
  -912,  -912,  -911,  -910,  -909,  -908,  -908,  -907,  -906,  -905,
  -904,  -903,  -903,  -902,  -901,  -900,  -899,  -898,  -897,  -897,
  -896,  -895,  -894,  -893,  -892,  -891,  -890,  -890,  -889,  -888,
  -887,  -886,  -885,  -884,  -883,  -882,  -881,  -881,  -880,  -879,
  -878,  -877,  -876,  -875,  -874,  -873,  -872,  -871,  -870,  -869,
  -868,  -868,  -867,  -866,  -865,  -864,  -863,  -862,  -861,  -860,
  -859,  -858,  -857,  -856,  -855,  -854,  -853,  -852,  -851,  -850,
  -849,  -848,  -847,  -846,  -845,  -844,  -843,  -842,  -841,  -840,
  -839,  -838,  -837,  -836,  -835,  -834,  -833,  -832,  -831,  -830,
  -829,  -827,  -826,  -825,  -824,  -823,  -822,  -821,  -820,  -819,
  -818,  -817,  -816,  -815,  -814,  -812,  -811,  -810,  -809,  -808,
  -807,  -806,  -805,  -804,  -803,  -801,  -800,  -799,  -798,  -797,
  -796,  -795,  -794,  -793,  -791,  -790,  -789,  -788,  -787,  -786,
  -785,  -783,  -782,  -781,  -780,  -779,  -778,  -776,  -775,  -774,
  -773,  -772,  -771,  -769,  -768,  -767,  -766,  -765,  -763,  -762,
  -761,  -760,  -759,  -757,  -756,  -755,  -754,  -753,  -751,  -750,
  -749,  -748,  -747,  -745,  -744,  -743,  -742,  -740,  -739,  -738,
  -737,  -735,  -734,  -733,  -732,  -730,  -729,  -728,  -727,  -725,
  -724,  -723,  -722,  -720,  -719,  -718,  -717,  -715,  -714,  -713,
  -711,  -710,  -709,  -708,  -706,  -705,  -704,  -702,  -701,  -700,
  -698,  -697,  -696,  -695,  -693,  -692,  -691,  -689,  -688,  -687,
  -685,  -684,  -683,  -681,  -680,  -679,  -677,  -676,  -675,  -673,
  -672,  -671,  -669,  -668,  -667,  -665,  -664,  -662,  -661,  -660,
  -658,  -657,  -656,  -654,  -653,  -651,  -650,  -649,  -647,  -646,
  -645,  -643,  -642,  -640,  -639,  -638,  -636,  -635,  -633,  -632,
  -631,  -629,  -628,  -626,  -625,  -624,  -622,  -621,  -619,  -618,
  -616,  -615,  -614,  -612,  -611,  -609,  -608,  -606,  -605,  -603,
  -602,  -601,  -599,  -598,  -596,  -595,  -593,  -592,  -590,  -589,
  -587,  -586,  -585,  -583,  -582,  -580,  -579,  -577,  -576,  -574,
  -573,  -571,  -570,  -568,  -567,  -565,  -564,  -562,  -561,  -559,
  -558,  -556,  -555,  -553,  -552,  -550,  -549,  -547,  -546,  -544,
  -543,  -541,  -540,  -538,  -537,  -535,  -534,  -532,  -531,  -529,
  -528,  -526,  -524,  -523,  -521,  -520,  -518,  -517,  -515,  -514,
  -512,  -511,  -509,  -508,  -506,  -504,  -503,  -501,  -500,  -498,
  -497,  -495,  -493,  -492,  -490,  -489,  -487,  -486,  -484,  -482,
  -481,  -479,  -478,  -476,  -475,  -473,  -471,  -470,  -468,  -467,
  -465,  -463,  -462,  -460,  -459,  -457,  -455,  -454,  -452,  -451,
  -449,  -447,  -446,  -444,  -443,  -441,  -439,  -438,  -436,  -435,
  -433,  -431,  -430,  -428,  -426,  -425,  -423,  -422,  -420,  -418,
  -417,  -415,  -413,  -412,  -410,  -408,  -407,  -405,  -404,  -402,
  -400,  -399,  -397,  -395,  -394,  -392,  -390,  -389,  -387,  -385,
  -384,  -382,  -380,  -379,  -377,  -375,  -374,  -372,  -370,  -369,
  -367,  -365,  -364,  -362,  -360,  -359,  -357,  -355,  -354,  -352,
  -350,  -349,  -347,  -345,  -344,  -342,  -340,  -339,  -337,  -335,
  -334,  -332,  -330,  -328,  -327,  -325,  -323,  -322,  -320,  -318,
  -317,  -315,  -313,  -312,  -310,  -308,  -306,  -305,  -303,  -301,
  -300,  -298,  -296,  -294,  -293,  -291,  -289,  -288,  -286,  -284,
  -282,  -281,  -279,  -277,  -276,  -274,  -272,  -270,  -269,  -267,
  -265,  -263,  -262,  -260,  -258,  -257,  -255,  -253,  -251,  -250,
  -248,  -246,  -244,  -243,  -241,  -239,  -237,  -236,  -234,  -232,
  -231,  -229,  -227,  -225,  -224,  -222,  -220,  -218,  -217,  -215,
  -213,  -211,  -210,  -208,  -206,  -204,  -203,  -201,  -199,  -197,
  -196,  -194,  -192,  -190,  -189,  -187,  -185,  -183,  -182,  -180,
  -178,  -176,  -174,  -173,  -171,  -169,  -167,  -166,  -164,  -162,
  -160,  -159,  -157,  -155,  -153,  -152,  -150,  -148,  -146,  -144,
  -143,  -141,  -139,  -137,  -136,  -134,  -132,  -130,  -129,  -127,
  -125,  -123,  -121,  -120,  -118,  -116,  -114,  -113,  -111,  -109,
  -107,  -105,  -104,  -102,  -100,   -98,   -97,   -95,   -93,   -91,
   -89,   -88,   -86,   -84,   -82,   -81,   -79,   -77,   -75,   -73,
   -72,   -70,   -68,   -66,   -64,   -63,   -61,   -59,   -57,   -56,
   -54,   -52,   -50,   -48,   -47,   -45,   -43,   -41,   -39,   -38,
   -36,   -34,   -32,   -31,   -29,   -27,   -25,   -23,   -22,   -20,
   -18,   -16,   -14,   -13,   -11,    -9,    -7,    -6,    -4,    -2,
    -0,
   };

#endif
