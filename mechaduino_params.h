//Contains the Mechaduino parameter declarations
//Copied from Mechaduino-Firmware project

#ifndef __PARAMS_H__
#define __PARAMS_H__

#define firmware_version "0.1.5"    //firmware version
#define identifier "x"              // change this to help keep track of multiple mechaduinos (printed on startup)

//----Current Parameters-----

extern volatile float Ts;
extern volatile float Fs;

extern volatile float pKp;
extern volatile float pKi;
extern volatile float pKd;
extern volatile float pLPF;


extern volatile float vKp;
extern volatile float vKi;
extern volatile float vKd;
extern volatile float vLPF;

extern const float lookup[];


extern volatile float pLPFa;
extern volatile float pLPFb;
extern volatile float vLPFa;
extern volatile float vLPFb;


extern const int spr; //  200 steps per revolution
extern float aps; // angle per step
extern int cpr; //counts per rev
extern float stepangle;

//extern volatile float PA;  //

extern const float iMAX;
extern const float rSense;
extern volatile int uMAX;


extern const int sin_1[];

//Defines for pins:

#define IN_4  ARDUINO_PIN_6
#define IN_3  ARDUINO_PIN_5
#define VREF_2 ARDUINO_PIN_4
#define VREF_1 ARDUINO_PIN_9
#define IN_2  ARDUINO_PIN_7
#define IN_1  ARDUINO_PIN_8
#define ledPin  ARDUINO_PIN_13
#define chipSelectPin ARDUINO_PIN_A2 //output to chip select

#define step_pin ARDUINO_PIN_1
#define dir_pin ARDUINO_PIN_0
#define enable_pin ARDUINO_PIN_2

//for faster digitalWrite:
#define IN_1_HIGH() (REG_PORT_OUTSET0 = PORT_PA06)
#define IN_1_LOW() (REG_PORT_OUTCLR0 = PORT_PA06)
#define IN_2_HIGH() (REG_PORT_OUTSET0 = PORT_PA21)
#define IN_2_LOW() (REG_PORT_OUTCLR0 = PORT_PA21)
#define IN_3_HIGH() (REG_PORT_OUTSET0 = PORT_PA15)
#define IN_3_LOW() (REG_PORT_OUTCLR0 = PORT_PA15)
#define IN_4_HIGH() (REG_PORT_OUTSET0 = PORT_PA20)
#define IN_4_LOW() (REG_PORT_OUTCLR0 = PORT_PA20)
#define ledPin_HIGH() (REG_PORT_OUTSET0 = PORT_PA17)
#define ledPin_LOW() (REG_PORT_OUTCLR0 = PORT_PA17)
#define CHIPSELECT_HIGH() (REG_PORT_OUTSET1 = PORT_PB09)
#define CHIPSELECT_LOW() (REG_PORT_OUTCLR1 = PORT_PB09)

#define ENABLE_PROFILE_IO    // Define to enable profiling I/O pins

#ifdef ENABLE_PROFILE_IO  
  #define TEST1   3

  #define TEST1_HIGH() (REG_PORT_OUTSET0 = PORT_PA09)
  #define TEST1_LOW() (REG_PORT_OUTCLR0 = PORT_PA09)

#else
  #define TEST1_HIGH()
  #define TEST1_LOW() 
#endif



#endif
