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

unsigned long lastReceivedPacket;
unsigned long lastLoopUpdate;
int loops;

struct GYRO_T gyro;
struct ACCL_T accl;
struct POLAR_T polar;
float temp;
float pressure;
float altitude;

void debugIMU(int debugMask);
void printDeviceId();
void calculateLoops();

void setup() {
  // Serial.begin(9600);
  // while(!Serial);

  comm.Init();
  imu.Init();
  motorControl.Init(0, 0.02, 0);

	lastReceivedPacket = 0;
  lastLoopUpdate = 0;
  loops = 0;

  imu.ReadGyro(&gyro);
  imu.ReadAccel(&accl);
  imu.ReadMagPolar(&polar);
}

void loop() {
  unsigned long ms = millis();
  unsigned char* packet = comm.ReceiveRadioPacket();

  imu.ReadGyro(&gyro);
  imu.ReadAccel(&accl);
  imu.ReadMagPolar(&polar);

  if (packet != NULL) {
    JoystickState state = {
      packet[3], packet[4], packet[5],
      packet[0], packet[1], packet[2]
    };

    motorControl.HandleJoystick(state);
    lastReceivedPacket = millis();
  }

  free(packet);

  // no lora packet received for a while, go into safety landing
  if (ms > lastReceivedPacket + 15000) {
    motorControl.SetMotorSpeed(0, 0, 0, 0);
  }
  else if (ms > lastReceivedPacket + 400) {
    // todo: set target roll/pitch and throttle
    motorControl.SetMotorSpeed(0.35, 0.35, 0.35, 0.35);
  }
  else {
    motorControl.CalculateTargetAngles(&accl, &gyro);
  }

  //calculateLoops();
  // temp = imu.ReadTempC();
  // pressure = imu.ReadPressurehPa();
  // altitude = imu.ConvPresToAltM(pressure);
  //debugIMU(DEBUG_GYRO | DEBUG_ACCL);
  //printDeviceId();
  // Serial.print(polar.R);
  // Serial.print(" ");
  // Serial.println(polar.D);
}


// DEBUG
void calculateLoops()
{
  loops++;

  if (millis() > lastLoopUpdate + 1000)
  {
    Serial.println(loops, DEC);
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
