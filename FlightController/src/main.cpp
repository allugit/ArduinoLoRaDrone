#include <Arduino.h>
#include <SPI.h>

#include <Joystick.h>
#include <Communication.h>
#include "MotorControl/MotorControl.h"
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
unsigned long lastLoopUpdate;
int loops;

void debugIMU(int debugMask);
void printDeviceId();
void calculateLoops();

void setup() {
  // Serial.begin(9600);
  // while(!Serial);

  motorControl.Init(0, 0, 0);
  imu.Init();
  comm.Init();

	lastReceivedPacket = 0;
  lastLoopUpdate = 0;
  loops = 0;

  imu.ReadGyro(&gyro);
  imu.ReadAccel(&accl);
}

void loop() {
  unsigned long ms = millis();
  unsigned char* packet = comm.ReceiveRadioPacket();

  if (packet != NULL) {
    JoystickState state = {
      packet[3], packet[4], packet[5],
      packet[0], packet[1], packet[2]
    };

    motorControl.HandleJoystick(state);
    lastReceivedPacket = millis();
  }

  free(packet);

  imu.ReadGyro(&gyro);
  imu.ReadAccel(&accl);

  // no lora packet received for a while, go into safety landing
  if (ms > lastReceivedPacket + 5000) {
    motorControl.SetMotorSpeed(0, 0, 0, 0);
  }
  else if (ms > lastReceivedPacket + 400) {
    // TODO: set target pitch and roll instead
    motorControl.SetMotorSpeed(0.3, 0.3, 0.3, 0.3);
  }
  else {
    motorControl.CalculateTargetAngles(&accl, &gyro);
  }

  calculateLoops();
  // temp = imu.ReadTempC();
  // pressure = imu.ReadPressurehPa();
  // altitude = imu.ConvPresToAltM(pressure);
  //debugIMU(DEBUG_GYRO | DEBUG_TEMP);
  //printDeviceId();
}


// DEBUG
void calculateLoops()
{
  loops++;

  if (millis() > lastLoopUpdate + 1000)
  {
    Serial.println(loops);
    loops = 0;
    lastLoopUpdate = millis();
  }
}

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
