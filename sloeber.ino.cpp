#ifdef __IN_ECLIPSE__
//This is a automatic generated file
//Please do not modify this file
//If you touch this file your change will be overwritten during the next build
//This file has been generated on 2020-06-29 14:59:54

#include "Arduino.h"
#include "Arduino.h"
#include "PID_v1.h"
#include "Wire.h"
#include "MotorController.h"
#include "string.h"

void encoderSignal() ;
void setMotorPwm(double value) ;
void stop() ;
void home() ;
void setPosTunings( double kp, double ki, double kd ) ;
void setSpeedTunings( double kp, double ki, double kd ) ;
void setMaxOutput(double maxOut) ;
void runAtSpeed(double speed) ;
void moveToPos(int pos) ;
void dumpState() ;
void twiRequest() ;
void twiReceive(int numBytes) ;
void serialReceive() ;
void calcSpeed(unsigned long timeChange) ;
void setup() ;
void loop() ;

#include "MotorController.ino"


#endif
