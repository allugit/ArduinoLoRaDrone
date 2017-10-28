#include "Joystick.h"

int LX_MAX = 1000;
int LX_MIN = 50;

void Joystick::Init()
{
    pinMode(JOYSTICK_LZ, INPUT);
    pinMode(JOYSTICK_LY, INPUT);
    pinMode(JOYSTICK_LX, INPUT);

    pinMode(JOYSTICK_RZ, INPUT);
    pinMode(JOYSTICK_RY, INPUT);
    pinMode(JOYSTICK_RX, INPUT);
}

/*
Roll – Done by pushing the right stick to the left or right.
       Literally rolls the quadcopter, which maneuvers the quadcopter left or right.
Pitch – Done by pushing the right stick forwards or backwards.
        Tilts the quadcopter, which maneuvers the quadcopter forwards or backwards.
Yaw – Done by pushing the left stick to the left or to the right.
      Rotates the quadcopter left or right. Points the front of the copter different directions and helps with changing directions while flying.
Throttle – To increase,  push the left stick forwards. To decrease, pull the left stick backwards.
           This adjusts the altitude, or height, of the quadcopter.
*/
JoystickState Joystick::GetJoystickAxises()
{
  int LX = analogRead(JOYSTICK_LX);
  int LY = analogRead(JOYSTICK_LY);
  int LZ = digitalRead(JOYSTICK_LZ);

  int RX = analogRead(JOYSTICK_RX);
  int RY = analogRead(JOYSTICK_RY);
  int RZ = digitalRead(JOYSTICK_RZ);

  ScaleAxisValues(&LX, &LY, &LZ);
  ScaleAxisValues(&RX, &RY, &RZ);

  // scale from 10 bit to 8 bit (1023 / 4 = 255)
  JoystickState state = {
	(unsigned char)(LX / 4), (unsigned char)(LY / 4), (unsigned char)LZ,
	(unsigned char)(RX / 4), (unsigned char)(RY / 4), (unsigned char)RZ
  };

  //DebugJoystick(state);
  return state;
}

void Joystick::DebugJoystick(JoystickState state)
{
    Serial.print(state.LX, DEC);
    Serial.print(", ");
    Serial.print(state.LY, DEC);
    Serial.print(", ");
    Serial.println(state.LZ, DEC);

	Serial.print(state.RX, DEC);
    Serial.print(", ");
    Serial.print(state.RY, DEC);
    Serial.print(", ");
    Serial.println(state.RZ, DEC);
}

void Joystick::ScaleAxisValues(int *x, int *y, int *z)
{
    *x = IsCenter(x) ? 500 : Clamp(*x, LX_MIN, LX_MAX);
    *y = IsCenter(y) ? 500 : Clamp(*y, LX_MIN, LX_MAX);
}

bool Joystick::IsCenter(int* axisValue)
{
    bool overUpperThreshold = *axisValue > JOYSTICK_ADC_CENTER + JOYSTICK_CENTER_THRESHOLD;
    bool overLowerThreshold = *axisValue < JOYSTICK_ADC_CENTER - JOYSTICK_CENTER_THRESHOLD;

    return !(overUpperThreshold || overLowerThreshold);
}

int Joystick::Clamp(int value, int min, int max)
{
    if (value > max) return max;
    else if (value < min) return min;
    else return value;
}
