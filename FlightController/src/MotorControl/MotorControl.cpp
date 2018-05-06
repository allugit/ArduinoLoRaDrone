#include "MotorControl.h"

// user controlled and current calculated pitch and roll
double targetPitch, currentPitch, controlPitch;
double targetRoll, currentRoll, controlRoll;
double targetYaw, currentYaw, controlYaw;
unsigned char targetThrottle;

// PID tunings: porpotional, integral, derivative
double Kp, Ki, Kd; // current values
double dKp, dKi, dKd; // default values
double dampenJoystickInput;
double* configIndexes[5];
unsigned char selectedConfigIndex;

double acclRatio;
float gyroOffsetX, gyroOffsetY, gyroOffsetZ;

unsigned long lastUpdate;
unsigned long nextPwmSetTime;
unsigned long flightCommandStart;
unsigned long configCommandStart;
int joystickCommandReceived;

int flightActive;
int joystickConnected;

/*Servo cwf;
Servo cwb;
Servo ccwf;
Servo ccwb;*/
Adafruit_PWMServoDriver pwm;

// input, output, setpoint, pid tunings (kp, ki, kd)
PID PIDPitch(&currentPitch, &controlPitch, &targetPitch, 0, 0, 0, DIRECT);
PID PIDRoll(&currentRoll, &controlRoll, &targetRoll, 0, 0, 0, DIRECT);
PID PIDYaw(&currentYaw, &controlYaw, &targetYaw, 0, 0, 0, DIRECT);

void MotorControl::Init(double kp, double ki, double kd)
{
  PIDPitch.SetMode(AUTOMATIC);
  PIDRoll.SetMode(AUTOMATIC);
  PIDYaw.SetMode(AUTOMATIC);

  PIDPitch.SetOutputLimits(-255, 255);
  PIDRoll.SetOutputLimits(-255, 255);
  PIDYaw.SetOutputLimits(-255, 255);

  PIDPitch.SetTunings(kp, ki, kd);
  PIDRoll.SetTunings(kp, ki, kd);
  PIDYaw.SetTunings(kp, ki, kd);

  PIDPitch.SetSampleTime(1);
  PIDRoll.SetSampleTime(1);
  PIDYaw.SetSampleTime(1);

  pwm.begin();
  pwm.setPWMFreq(PWM_FREQ);

  currentPitch = 0;
  currentRoll = 0;
  currentYaw = 0;
  controlPitch = 0;
  controlRoll = 0;
  controlYaw = 0;

  targetThrottle = 0;
  targetPitch = 0;
  targetRoll = 0;
  targetYaw = 0;

  // for joystick PID configuration
  dKp = Kp = kp;
  dKi = Ki = ki;
  dKd = Kd = kd;
  configIndexes[0] = &Kp;
  configIndexes[1] = &Ki;
  configIndexes[2] = &Kd;
  configIndexes[3] = &dampenJoystickInput;
  configIndexes[4] = &acclRatio;

  dampenJoystickInput = 0.5f;
  acclRatio = DEFAULT_ACCL_RATIO;
  gyroOffsetX = gyroOffsetY = gyroOffsetZ = 0;

  digitalWrite(LED_BUILTIN, HIGH);
  SetMotorSpeed(0, 0, 0, 0);
  delay(3000);
  digitalWrite(LED_BUILTIN, LOW);

  flightActive = 0;
  joystickConnected = 0;
  nextPwmSetTime = 0;
  flightCommandStart = 0;
  configCommandStart = 0;
  joystickCommandReceived = 0;
  selectedConfigIndex = 0;
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
  joystickConnected = 1;

  // throttle is controlled directly
  targetThrottle = state.LY;
  targetYaw = 0;

   if (targetThrottle < JOYSTICK_MIN_THROTTLE) {
     targetThrottle = 0;
   }

  // calculate pitch and roll direction
  // pitch and roll are calculated as +/- degrees of copter angle (because they're returned like that from IMU)
  // and inputed to PID-controller, output is -255 to 255 which is summed with targetThrottle
  // joystick state is a byte 0-255
  // scale against max pitch and roll
  targetPitch =  dampenJoystickInput * (state.RY * (JOYSTICK_PITCH_LIMIT * 2.0f / 255.0f) - JOYSTICK_PITCH_LIMIT);
  targetRoll = dampenJoystickInput * (state.RX * (JOYSTICK_ROLL_LIMIT * 2.0f / 255.0f) - JOYSTICK_ROLL_LIMIT);
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
    if (targetThrottle < JOYSTICK_MIN_THROTTLE) {
      flightActive = 1;
    }
    else {
      HandleJoystickCommands(&state);
    }
  }
}

void MotorControl::HandleJoystickCommands(JoystickState* state)
{
  if (joystickCommandReceived) {
    HandleCommand(state);
  }
  else {
    PollCommand(state);
  }
}

void MotorControl::HandleCommand(JoystickState* state)
{
  int xCenter = state->RX < DIR_LEFT && state->RX > DIR_RIGHT;
  int yCenter = state->RY > DIR_DOWN && state->RY < DIR_UP;

  if (xCenter && yCenter) {
    PIDPitch.SetTunings(Kp, Ki, Kd);
    PIDRoll.SetTunings(Kp, Ki, Kd);
    PIDYaw.SetTunings(Kp, Ki, Kd);

    // Serial.println(selectedConfigIndex % 4);
    // Serial.println(acclRatio);
    // Serial.print(PIDPitch.GetKp());
    // Serial.print(" ");
    // Serial.print(PIDPitch.GetKi());
    // Serial.print(" ");
    // Serial.println(PIDPitch.GetKd());
    // Serial.print(PIDRoll.GetKp());
    // Serial.print(" ");
    // Serial.print(PIDRoll.GetKi());
    // Serial.print(" ");
    // Serial.println(PIDRoll.GetKd());

    if (joystickCommandReceived == 6) {
      BlinkLed(3000, 1);
    }
    else {
      BlinkLed(250, joystickCommandReceived);
    }

    joystickCommandReceived = 0;
  }
}

void MotorControl::PollCommand(JoystickState* state)
{
  if (state->RX > DIR_LEFT) {
    if (WaitForHoldCommand(&configCommandStart, 2000)) {
      Kp = dKp; Ki = dKi; Kd = dKd;
      dampenJoystickInput = 0.5f;
      acclRatio = DEFAULT_ACCL_RATIO;
      currentRoll = 0;
      currentPitch = 0;
      joystickCommandReceived = 6;
    }
  }
  else {
    // change config value
    if (state->RX < DIR_RIGHT) {
      selectedConfigIndex++;
      joystickCommandReceived = (selectedConfigIndex % 5) + 1;
    }
    // increment config value
    else if (state->RY > DIR_UP) {
      double increment = selectedConfigIndex % 5 == 4 ? ACCL_RATIO_INCREMENT : PID_INCREMENT;
      *configIndexes[selectedConfigIndex % 5] += increment;
      joystickCommandReceived = 2;
    }
    // decrement config value
    else if (state->RY < DIR_DOWN) {
      if (*configIndexes[selectedConfigIndex % 5] > 0) {
        double increment = selectedConfigIndex % 5 == 4 ? ACCL_RATIO_INCREMENT : PID_INCREMENT;
        *configIndexes[selectedConfigIndex % 5] -= increment;
        joystickCommandReceived = 1;
      }
    }
  }

  // cancel command?
  if (state->LZ && state->RX < DIR_LEFT) {
    configCommandStart = 0;
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
  double deltaTime = (double)(millis() - lastUpdate) / 1000;

  // AcclAngle(accl); // using only accelerometer has lots of noise
  // 1. use complementary filter to filter out accelerometer noise
  ComplementaryFilter(accl, gyro, deltaTime);
  //DebugOrientation ();

  // AcclAngle(accl);
  // DebugOrientation();

  // 2. use PID controller to control output value for motors
  PIDPitch.Compute();
  PIDRoll.Compute();
  PIDYaw.Compute();

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
  //unsigned long now = millis();

  if (targetThrottle > 0)
  {
    float cp = controlPitch;
    float cr = controlRoll;
    float cy = 0;//controlYaw;

    int cwf = targetThrottle - cp - cr - cy;
    int ccwf = targetThrottle - cp + cr + cy;
    int ccwb = targetThrottle + cp + cr + cy;
    int cwb = targetThrottle + cp - cr - cy;

    int max = 255;

    if (cwf > max) max = cwf;
    if (ccwf > max) max = ccwf;
    if (ccwb > max) max = ccwb;
    if (cwb > max) max = cwb;

    int overLimit = max - 255;

    if (overLimit > 0)
    {
        cwf -= overLimit;
        ccwf -= overLimit;
        ccwb -= overLimit;
        cwb -= overLimit;
    }

    cwf = Clamp(cwf, 0, 255);
    ccwf = Clamp(ccwf, 0, 255);
    ccwb = Clamp(ccwb, 0, 255);
    cwb = Clamp(cwb, 0, 255);

    SetMotorSpeed(cwf / 255.0f, cwb / 255.0f, ccwf / 255.0f, ccwb / 255.0f);
  }
  else
  {
    SetMotorSpeed(0, 0, 0, 0);
  }
}

void MotorControl::SetMotorSpeed(float scwf, float scwb, float sccwf, float sccwb)
{
  if (!flightActive) {
    pwm.setPWM(MOTOR_CWF, 0, 4096);
    pwm.setPWM(MOTOR_CWB, 0, 4096);
    pwm.setPWM(MOTOR_CCWF, 0, 4096);
    pwm.setPWM(MOTOR_CCWB, 0, 4096);
  }
  else
  {
    pwm.setPWM(MOTOR_CWF, 0, 4090 * scwf);
    pwm.setPWM(MOTOR_CWB, 0, 4090 * scwb);
    pwm.setPWM(MOTOR_CCWF, 0, 4090 * sccwf);
    pwm.setPWM(MOTOR_CCWB, 0, 4090 * sccwb);
   }
}

// http://www.pieter-jan.com/node/11
// currentAngle = (1 - lowPassRatio) * (currentAngle + gyro * dt) + lowPassRatio * (accl)
void MotorControl::ComplementaryFilter(struct ACCL_T *accl, struct GYRO_T *gyro, double deltaTime)
{
  // integrate gyro values
  currentRoll += GYRO_SENSITIVITY * (gyro->X - gyroOffsetX) * deltaTime;
  currentPitch -= GYRO_SENSITIVITY * (gyro->Y - gyroOffsetY) * deltaTime;
  // currentYaw += GYRO_SENSITIVITY * (gyro->Z - gyroOffsetZ) * deltaTime;

  // compensate drift with accel data, ignore large magnitudes
  // if the accelerometer data is within a 0.5-2G, then we will use that data
  float forceMagApprox = abs(accl->X) + abs(accl->Y) + abs(accl->Z);

  //
  if (forceMagApprox < 2) {
    float rollAccl = -atan2f(accl->Y, accl->Z) * 57.2957;
    float pitchAccl = -atan2f(accl->X, accl->Z) * 57.2957;
    //float yawAccl = atan(accl->Z/sqrt(accl->X*accl->X + accl->Z*accl->Z)) * 57.2957;

    currentPitch = (1 - acclRatio) * currentPitch + pitchAccl * acclRatio;
    currentRoll = (1 - acclRatio) * currentRoll + rollAccl * acclRatio;
    //currentYaw = (1 - acclRatio) * currentYaw + yawAccl * acclRatio;
  }
}

// don't use during flight as accelometer has a lot of noise
void MotorControl::AcclAngle(struct ACCL_T *accl)
{
  // currentPitch = atan2f(accl->Y, sqrt(accl->X * accl->X + accl->Z * accl->Z)) * 57.2957;
  // currentRoll = atan2f(-accl->X, accl->Z) * 57.2957;

  currentPitch = -atan2f(accl->X, accl->Z) * 57.2957;
  currentRoll = -atan2f(accl->Y, accl->Z) * 57.2957;
}

// minor detail: with pointers it is faster because value does not need to be copied
int MotorControl::Clamp(int val, int min, int max)
{
  if (val < min) return min;
  if (val > max) return max;
  return val;
}

void MotorControl::SetGyroOffset(float x, float y, float z)
{
  gyroOffsetX = x;
  gyroOffsetY = y;
  gyroOffsetZ = z;
}

void MotorControl::DebugOrientation()
{
  // for (int i = 0; i < 5; i++)
  // {
  //   Serial.print(*configIndexes[i]);
  //   Serial.print(", ");
  // }
  // Serial.println();

  // Serial.print(currentPitch);
  // Serial.print(" : ");
  Serial.print(controlPitch);
  Serial.print(" : ");
  // Serial.print(targetPitch);
  // Serial.print(" , ");
  // //
  // Serial.print(currentRoll);
  // Serial.print(" : ");
  // Serial.println();

  Serial.print(controlRoll);
  // Serial.print(" : ");
  // Serial.print(targetRoll);
  Serial.println();

  // Serial.print(cwf.read());
  // Serial.print(", ");
  // Serial.print(ccwf.read());
  // Serial.print("; ");
  // Serial.print(cwb.read());
  // Serial.print(" , ");
  // Serial.println(ccwb.read());

  //Serial.println(flightActive);

  // Serial.print(gyro->X);
  // Serial.print(" : ");
  // Serial.println(gyro->X * deltaTime);
  // Serial.print(gyro->Y - 2);
  // Serial.print(" : ");
  // Serial.println((gyro->Y - 2) * deltaTime);
  // Serial.println("");
}
