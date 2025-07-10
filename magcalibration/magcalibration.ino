/**
* This logs magnetometer data for calibration purposes
*/

#include <Arduino.h>
#include <Wire.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <SD.h>
#include <SPI.h>
#include <SensorIMU.h>
#include <Logger.h>
#include <Printer.h>
#define UartSerial Serial1
#define LOOP_PERIOD 15
#define IMU_LOOP_OFFSET 1
#define LOGGER_LOOP_OFFSET 2
#define PRINTER_LOOP_OFFSET 13

SensorIMU imu;
Logger logger;
Printer printer;
int loopStartTime;
int currentTime;
int i;

void setup() {
  i = 0;
  logger.include(&imu);
  logger.init();
  printer.init();
  imu.init();

  printer.printMessage("Starting main loop",10);
  loopStartTime = millis();
  printer.lastExecutionTime            = loopStartTime - LOOP_PERIOD + PRINTER_LOOP_OFFSET ;
  imu.lastExecutionTime                = loopStartTime - LOOP_PERIOD + IMU_LOOP_OFFSET;
  logger.lastExecutionTime             = loopStartTime - LOOP_PERIOD + LOGGER_LOOP_OFFSET;
  delay(2000);
}

void loop() {
  // put your main code here, to run repeatedly:
  currentTime=millis();

  if ( currentTime-printer.lastExecutionTime > LOOP_PERIOD ) {
    i++;
    Serial.println(i);
    printer.lastExecutionTime = currentTime;
    printer.printValue(1,logger.printState()); 
    printer.printValue(2,imu.printRollPitchHeading());        
    printer.printValue(3,imu.printAccels());
    printer.printToSerial();  // To stop printing, just comment this line out
  }

  if ( currentTime-imu.lastExecutionTime > LOOP_PERIOD ) {
    imu.lastExecutionTime = currentTime;
    imu.read();     // blocking I2C calls
  }

  if ( currentTime- logger.lastExecutionTime > LOOP_PERIOD && logger.keepLogging ) {
    logger.lastExecutionTime = currentTime;
    logger.log();
  }
}
