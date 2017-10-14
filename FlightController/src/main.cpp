#include <Arduino.h>
#include <SPI.h>

#include "MotorControl/MotorControl.h"
#include "Communication/Communication.h"
#include "IMU/IMU.h"

#define DEBUG_ACCL 1
#define DEBUG_GYRO 2
#define DEBUG_ALT  4
#define DEBUG_TEMP 8
#define LED 13

Communication comm;
MotorControl motorControl;
IMU imu;

struct GYRO_T gyro;
struct ACCL_T accl;
float temp;
float pressure;
float altitude;

unsigned long lastReceivedPacket;
unsigned long lastIMUHandle;
int handleIMU;
unsigned long imuIndex;
int handleCommunication;

// DEBUG
void debugIMU(int debugMask)
{
  if (debugMask & DEBUG_ACCL)
  {
    //Serial.println("Accel (x, y, z): ");
    Serial.print(accl.X);
    Serial.print(", ");
    Serial.print(accl.Y);
    Serial.print(", ");
    Serial.println(accl.Z);
    Serial.println("");
  }

  if (debugMask & DEBUG_GYRO)
  {
    Serial.println("Gyro (x, y, z): ");
    Serial.print(gyro.X);
    Serial.print(", ");
    Serial.print(gyro.Y);
    Serial.print(", ");
    Serial.println(gyro.Z);
    Serial.println("");
  }

  if (debugMask & DEBUG_ALT)
  {
    Serial.print("Altitude (meters): ");
    Serial.println(altitude);
    Serial.print("Pressure (hPa): ");
    Serial.println(pressure);
    Serial.println("");
  }

  if (debugMask & DEBUG_TEMP)
  {
    Serial.print("Temperature C: ");
    Serial.println(temp);
    Serial.println("");
  }
}

void printDeviceId()
{
  struct DEV_ID id;

  imu.GetDeviceID(&id);
  Serial.println(id.ag);
  Serial.println(id.mag);
  Serial.println(id.alt);
}

void setup() {
  // Serial.begin(9600);
  // while(!Serial);

  motorControl.Init(2, 5, 1);
  imu.Init();
  comm.Init();

  handleIMU = 1;
  handleCommunication = 1;
	lastReceivedPacket = 0;
  lastIMUHandle = 0;

  imu.ReadGyro(&gyro);
  imu.ReadAccel(&accl);

  //Serial.println("Init done");
}

void loop() {
  unsigned long ms = millis();
  // delay(100);
  // Serial.println("asd");

  if (handleCommunication) {
    unsigned char* packet = comm.ReceiveRadioPacket();

    if (packet != NULL) {
      motorControl.HandleJoystick(packet[0], packet[1], packet[3], packet[4], packet[2]);
      lastReceivedPacket = millis();
    }

    free(packet);
  }

  imu.ReadGyro(&gyro);
  imu.ReadAccel(&accl);

  // no lora packet received for a while, go into safety landing
  if (ms > lastReceivedPacket + 5000) {
    motorControl.SetMotorSpeed(0, 0, 0, 0);
  }
  else if (ms > lastReceivedPacket + 400) {
    motorControl.SetMotorSpeed(0.3, 0.3, 0.3, 0.3);
  }
  else {
    motorControl.CalculateTargetAngles(&accl, &gyro);
  }

  // temp = imu.ReadTempC();
  // pressure = imu.ReadPressurehPa();
  // altitude = imu.ConvPresToAltM(pressure);
  //debugIMU(DEBUG_GYRO | DEBUG_TEMP);
  //printDeviceId();
}
