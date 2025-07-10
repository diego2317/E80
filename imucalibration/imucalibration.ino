#include <Arduino.h>
#include <Wire.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <SD.h>
#include <SPI.h>
#include <Adafruit_LSM6DS3TRC.h>
#include <SensorIMU.h>
#include <OtherIMU.h>
#include <Logger.h>
#include <Printer.h>
#define LOOP_PERIOD 50
#define BOARD_IMU_OFFSET 1
#define IMU_1_OFFSET 2
#define IMU_2_OFFSET 3
#define LOGGER_LOOP_OFFSET 4
#define PRINTER_LOOP_OFFSET 14

SensorIMU imu;
Logger logger;
Printer printer;
OtherIMU otherIMU_1;
OtherIMU otherIMU_2 = OtherIMU(2);

int loopStartTime;
int currentTime;
int i;

void setup() {
  i = 0;
  logger.include(&imu);
  logger.include(&otherIMU_1);
  logger.include(&otherIMU_2);
  logger.init();

  printer.init();
  imu.init();
  otherIMU_1.init(1);
  otherIMU_2.init(2);

  printer.lastExecutionTime            = loopStartTime - LOOP_PERIOD + PRINTER_LOOP_OFFSET;
  imu.lastExecutionTime                = loopStartTime - LOOP_PERIOD + BOARD_IMU_OFFSET;
  logger.lastExecutionTime             = loopStartTime - LOOP_PERIOD + LOGGER_LOOP_OFFSET;
  otherIMU_1.lastExecutionTime         = loopStartTime - LOOP_PERIOD + IMU_1_OFFSET;
  otherIMU_2.lastExecutionTime         = loopStartTime - LOOP_PERIOD + IMU_2_OFFSET;
  delay(5000);

}

void loop() {
  // put your main code here, to run repeatedly:
  currentTime=millis();
  if ( currentTime-printer.lastExecutionTime > LOOP_PERIOD ) {
    i++;
    Serial.println(i);
    printer.lastExecutionTime = currentTime;
    printer.printValue(1,logger.printState()); 
    printer.printValue(2, otherIMU_1.printAccels());
    printer.printValue(3, otherIMU_2.printAccels());
    //printer.printValue(8,motor_driver.printState());
    //printer.printValue(9,imu.printRollPitchHeading());        
    printer.printValue(4,imu.printAccels());
    //printer.printValue(11, calibrationMessage,20);
    printer.printToSerial();  // To stop printing, just comment this line out
  }

  if ( currentTime-imu.lastExecutionTime > LOOP_PERIOD ) {
    imu.lastExecutionTime = currentTime;
    imu.read();     // blocking I2C calls
  }

  if (currentTime - otherIMU_1.lastExecutionTime > LOOP_PERIOD) {
    otherIMU_1.lastExecutionTime = currentTime;
    otherIMU_1.read();
  }

  if (currentTime - otherIMU_2.lastExecutionTime > LOOP_PERIOD) {
   otherIMU_2.lastExecutionTime = currentTime;
   otherIMU_2.read();
  }

  if ( currentTime- logger.lastExecutionTime > LOOP_PERIOD && logger.keepLogging ) {
    logger.lastExecutionTime = currentTime;
    logger.log();
  }
}
