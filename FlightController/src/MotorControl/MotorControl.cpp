#include "MotorControl.h"

// user controlled and current calculated pitch and roll
double targetPitch, currentPitch, controlPitch;
double targetRoll, currentRoll, controlRoll;
unsigned char targetThrottle;

// PID tunings: porpotional, integral, derivative
double Kp, Ki, Kd; // current values
double dKp, dKi, dKd; // default values
double* pidIndexes[3];
double acclRatio;

unsigned long lastUpdate;
unsigned long flightCommandStart;
unsigned long configCommandStart;
unsigned char selectedPidIndex;
int joystickCommandHandled;

int flightActive;

Servo cwf;
Servo cwb;
Servo ccwf;
Servo ccwb;

// input, output, setpoint, pid tunings (kp, ki, kd)
PID PIDPitch(&currentPitch, &controlPitch, &targetPitch, 0, 0, 0, DIRECT);
PID PIDRoll(&currentRoll, &controlRoll, &targetRoll, 0, 0, 0, DIRECT);

void MotorControl::Init(double kp, double ki, double kd)
{
  PIDPitch.SetMode(AUTOMATIC);
  PIDRoll.SetMode(AUTOMATIC);

  PIDPitch.SetOutputLimits(-30, 30);
  PIDRoll.SetOutputLimits(-30, 30);

  PIDPitch.SetTunings(kp, ki, kd);
  PIDRoll.SetTunings(kp, ki, kd);

  // 5 ms sample time, 200 Hz
  // this should be fast enough
  PIDPitch.SetSampleTime(5);
  PIDRoll.SetSampleTime(5);

  // attach motor pins to servo object
  cwf.attach(MOTOR_CWF);
  cwb.attach(MOTOR_CWB);
  ccwf.attach(MOTOR_CCWF);
  ccwb.attach(MOTOR_CCWB);

  currentPitch = 0;
  currentRoll = 0;
  controlPitch = 0;
  controlRoll = 0;

  targetThrottle = 255;
  targetPitch = 0;
  targetRoll = 0;

  // for joystick PID configuration
  dKp = Kp = kp;
  dKi = Ki = ki;
  dKd = Kd = kd;
  pidIndexes[0] = &Kp;
  pidIndexes[1] = &Ki;
  pidIndexes[2] = &Kd;

  acclRatio = DEFAULT_ACCL_RATIO;

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  SetMotorSpeed(0,0,0,0);
  delay(3000);

  digitalWrite(LED_BUILTIN, LOW);

  flightActive = 0;
  flightCommandStart = 0;
  configCommandStart = 0;
  joystickCommandHandled = 0;
  selectedPidIndex = 0;
  lastUpdate = millis();
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
void MotorControl::HandleJoystick(JoystickState state)
{
  // throttle is controlled directly
  targetThrottle = state.LY;

  // calculate pitch and roll direction
  // pitch and roll are calculated as +/- degrees of copter angle (because they're returned like that from IMU)
  // and inputed to PID-controller, output is -255 to 255 which is summed with targetThrottle
  // joystick state is a byte 0-255
  // scale against max pitch and roll
  targetPitch = state.RY * (PITCH_LIMIT * 2.0f / 255.0f) - PITCH_LIMIT;
  targetRoll = state.RX * (ROLL_LIMIT * 2.0f / 255.0f) - ROLL_LIMIT;
  if (abs(targetPitch) < 1) targetPitch = 0;
  if (abs(targetRoll) < 1) targetRoll = 0;

  // hold right stick 1s to turn off motors
  if (state.RZ) {
    flightCommandStart = 0;
  }
  else {
    if (WaitForHoldCommand(&flightCommandStart, 1000)) {
      flightActive = 0;
    }
  }

  if (!flightActive) {
    // activate motors only when user throttle minimum
    if (targetThrottle < 15) {
      flightActive = 1;
    }
    else {
      HandleJoystickCommands(&state);
    }
  }
}

void MotorControl::HandleJoystickCommands(JoystickState* state)
{
  if (joystickCommandHandled) {
    int xCenter = state->RX < DIR_LEFT && state->RX > DIR_RIGHT;
    int yCenter = state->RY > DIR_DOWN && state->RY < DIR_UP;

    if (xCenter && yCenter) {
      PIDPitch.SetTunings(Kp, Ki, Kd);
      PIDRoll.SetTunings(Kp, Ki, Kd);

      Serial.println(selectedPidIndex % 3);
      Serial.println(acclRatio);
      Serial.print(PIDPitch.GetKp());
      Serial.print(" ");
      Serial.print(PIDPitch.GetKi());
      Serial.print(" ");
      Serial.println(PIDPitch.GetKd());
      Serial.print(PIDRoll.GetKp());
      Serial.print(" ");
      Serial.print(PIDRoll.GetKi());
      Serial.print(" ");
      Serial.println(PIDRoll.GetKd());

      if (joystickCommandHandled == 5) {
        BlinkLed(3000, 1);
      }
      else {
        BlinkLed(250, joystickCommandHandled);
      }

      joystickCommandHandled = 0;
    }
  }
  else {
    if (!state->LZ) {
      if (WaitForHoldCommand(&configCommandStart, 2000)) {
        acclRatio = acclRatio == DEFAULT_ACCL_RATIO ? 0 : DEFAULT_ACCL_RATIO;
        currentRoll = 0;
        currentPitch = 0;
        joystickCommandHandled = 4;
      }
    }
    else if (state->RX > DIR_LEFT) {
      if (WaitForHoldCommand(&configCommandStart, 2000)) {
        Kp = dKp; Ki = dKi; Kd = dKd;
        joystickCommandHandled = 5;
      }
    }
    else {
      if (state->RX < DIR_RIGHT) {
        selectedPidIndex++;
        joystickCommandHandled = (selectedPidIndex % 3) + 1;
      }
      else if (state->RY > DIR_UP) {
        *pidIndexes[selectedPidIndex % 3] += PID_INCREMENT;
        joystickCommandHandled = 2;
      }
      else if (state->RY < DIR_DOWN) {
        if (*pidIndexes[selectedPidIndex % 3] > 0) {
          *pidIndexes[selectedPidIndex % 3] -= PID_INCREMENT;
          joystickCommandHandled = 1;
        }
      }
    }

    if (state->LZ && state->RX < DIR_LEFT) {
      configCommandStart = 0;
    }
  }
}

int MotorControl::WaitForHoldCommand(unsigned long* start, int holdTime)
{
  if (*start == 0) {
    *start = millis();
  }
  else if (millis() > *start + holdTime) {
    *start = 0;
    return 1;
  }

  return 0;
}

void MotorControl::BlinkLed(int blinkTime, int count)
{
  int i, state;

  for (i = 0, state = 1; i < count * 2; i++)
  {
    digitalWrite(LED_BUILTIN, state);
    state = !state;
    delay(blinkTime);
  }
}

void MotorControl::CalculateTargetAngles(struct ACCL_T *accl, struct GYRO_T *gyro)
{
  float deltaTime = (float)(millis() - lastUpdate) / 1000;

  // AcclAngle(accl); // using only accelerometer has lots of noise
  // 1. use complementary filter to filter out accelerometer noise
  ComplementaryFilter(accl, gyro, deltaTime);

  // 2. use PID controller to control output value for motors
  PIDPitch.Compute();
  PIDRoll.Compute();

  // 3. calculate new motor speed
  UpdateMotorSpeed();

  lastUpdate = millis();
}

// + copter
// Front = Throttle + PitchPID - YawPID
// Back = Throttle - PitchPID - YawPID
// Left = Throttle + RollPID + YawPID
// Right = Throttle - RollPID + YawPID

// x copter
// FrontL = Throttle + Pitch + Yaw
// FrontR = Throttle + Pitch - Yaw
// BackL = Throttle - Pitch - Yaw
// BackR = Throttle - Pitch + Yaw
void MotorControl::UpdateMotorSpeed()
{
	// is the controlRoll/pitch values wrong btw?
	// is it actually degrees (-30...30) scaled to -255...255
	// it works pretty well during simulation anyway and maybe it is ok to control like this?
  int CWF = targetThrottle - controlPitch - controlRoll; // - Yaw
  int CCWF = targetThrottle - controlPitch + controlRoll; // + Yaw
  int CCWB = targetThrottle + controlPitch + controlRoll;  // + Yaw
  int CWB = targetThrottle + controlPitch - controlRoll;  // - Yaw

  // find largest over limit value -255...255
  int max = 255;

  if (abs(CWF) > max) max = CWF;
  if (abs(CCWF) > max) max = CCWF;
  if (abs(CCWB) > max) max = CCWB;
  if (abs(CWB) > max) max = CWB;

  int overLimit = abs(max) - 255;
  overLimit = max < 0 ? overLimit : -overLimit;

  if (abs(overLimit) > 0)
  {
      CWF = Clamp(CWF + overLimit, 0, 255);
      CCWF = Clamp(CCWF + overLimit, 0, 255);
      CCWB = Clamp(CCWB + overLimit, 0, 255);
      CWB = Clamp(CWB + overLimit, 0, 255);
  }

  SetMotorSpeed(CWF / 255.0f, CWB / 255.0f, CCWF / 255.0f, CCWB / 255.0f);
  //SetMotorSpeed(targetThrottle / 255.0f, targetThrottle / 255.0f, targetThrottle / 255.0f, targetThrottle / 255.0f);
}

void MotorControl::SetMotorSpeed(float scwf, float scwb, float sccwf, float sccwb)
{
  if (!flightActive) {
    cwf.write(0);
    cwb.write(0);
    ccwf.write(0);
    ccwb.write(0);
  }
  else {
    // most RC transmitters put out a smaller range of around 1.2~1.8 ms
    // 180 = 2 ms and motors will shutdown
    // should scale throttle value to allow using the whole range of joystick,
    // but quick fix for now
    // 1.8 / 2 = 0.9 but still shuts down
    cwf.write(180 * (scwf > 0.85 ? 0.85 : scwf));
    cwb.write(180 * (scwb > 0.85 ? 0.85 : scwb));
    ccwf.write(180 * (sccwf > 0.85 ? 0.85 : sccwf));
    ccwb.write(180 * (sccwb > 0.85 ? 0.85 : sccwb));

    Serial.print(currentPitch);
    Serial.print(" : ");
    Serial.print(controlPitch);
    Serial.print(" : ");
    Serial.print(targetPitch);
    Serial.print(" , ");

    Serial.print(currentRoll);
    Serial.print(" : ");
    Serial.print(controlRoll);
    Serial.print(" : ");
    Serial.print(targetRoll);
    Serial.println("");

    // Serial.print(gyro->X);
    // Serial.print(" : ");
    // Serial.println(gyro->X * deltaTime);
    // Serial.print(gyro->Y - 2);
    // Serial.print(" : ");
    // Serial.println((gyro->Y - 2) * deltaTime);
    // Serial.println("");
  }

  // Serial.print(scwf);
  // Serial.print(", ");
  // Serial.print(sccwf);
  // Serial.print("; ");
  // Serial.print(scwb);
  // Serial.print(" , ");
  // Serial.println(sccwb);
}

// http://www.pieter-jan.com/node/11
// currentAngle = (1 - lowPassRatio) * (currentAngle + gyro * dt) + lowPassRatio * (accl)
void MotorControl::ComplementaryFilter(struct ACCL_T *accl, struct GYRO_T *gyro, float deltaTime)
{
  // integrate gyro values
  currentRoll += gyro->X * deltaTime;
  currentPitch -= (gyro->Y - 2) * deltaTime; // -2 remove bias, maybe add calibration later on?

  // compensate drift with accel data, ignore large magnitudes
  // if the accelerometer data is within a 0.5-2G, then we will use that data
  int forceMagApprox = abs(accl->X) + abs(accl->Y) + abs(accl->Z);

  if (forceMagApprox > 0.5 && forceMagApprox < 1.5) {
    float rollAccl = atan2f(accl->Y, accl->Z) * 57.2957;
    float pitchAccl = atan2f(-accl->X, accl->Z) * 57.2957;

    currentPitch = (1 - acclRatio) * currentPitch + pitchAccl * acclRatio;
    currentRoll = (1 - acclRatio) * currentRoll + rollAccl * acclRatio;
  }
}

// don't use during flight as accelometer has a lot of noise
void MotorControl::AcclAngle(struct ACCL_T *accl)
{
  currentPitch = atan2f(accl->Y, sqrt(accl->X * accl->X + accl->Z * accl->Z)) * 57.2957;
  currentRoll = atan2f(-accl->X, accl->Z) * 57.2957;
}

void MotorControl::LimitAngles(double *pitch, double *roll)
{
  if (abs(*pitch) > PITCH_LIMIT)
  {
    int sign = *pitch < 0 ? -1 : 1;
    *pitch = PITCH_LIMIT * sign;
  }

  if (abs(*roll) > ROLL_LIMIT)
  {
    int sign = *roll < 0 ? -1 : 1;
    currentRoll = ROLL_LIMIT * sign;
  }
}

// minor detail: with pointers it is faster because value does not need to be copied
int MotorControl::Clamp(int val, int min, int max)
{
  if (val < min) return min;
  if (val > max) return max;
  return val;
}



// // timer 2 (controls pin 10, 9)
// pinMode(MOTOR_CW1, OUTPUT);
// pinMode(MOTOR_CW2, OUTPUT);
//
// // timer 1 (controls pin 12, 11)
// pinMode(MOTOR_CCW1, OUTPUT);
// pinMode(MOTOR_CCW2, OUTPUT);
//
// // analog joystick pins
// pinMode(JOYSTICK_LZ, INPUT);
// pinMode(JOYSTICK_LY, INPUT);
// pinMode(JOYSTICK_LX, INPUT);
//
// // Timer/counter control register 1 and 2
// // Fast PWM, Clear OC2A on Compare Match, set OC2A at BOTTOM (non-inverting mode)
// TCCR1A = _BV(COM1A1) | _BV(COM1B1) | _BV(WGM12) | _BV(WGM10);
// TCCR2A = _BV(COM2A1) | _BV(COM2B1) | _BV(WGM21) | _BV(WGM20);
//
// // // 100 => 16 Mhz / 64 / 256 = 976 Hz
// // // 111 => 16 Mhz / 1024 / 256 = 61 Hz
// // // TCCR2 and other TCCR's are different, maybe should not use TCCR2
// // // TCCR2B = _BV(CS22) | _BV(CS21) | _BV(20);
// // TCCR2B = _BV(CS22);
// // TCCR1B = _BV(CS11) | _BV(CS10);
//
// // 16 Mhz / 1024 / 256 = 61 Hz
// TCCR2B = _BV(CS22) | _BV(CS21) | _BV(CS20);
// TCCR1B = _BV(CS12);

// 8 bit output compare register A and B
// (val+1) / 256 = duty cycle (%);
// OCR1A = (byte)(m1 * 256);
// OCR1B = (byte)(m2 * 256);
// OCR2A = (byte)(m3 * 256);
// OCR2B = (byte)(m4 * 256);
