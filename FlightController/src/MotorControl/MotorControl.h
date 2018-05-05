#ifndef MotorControl_h
#define MotorControl_h

#include <Arduino.h>
#include <Servo.h>
#include <PID_v1.h>
#include <Adafruit_PWMServoDriver.h>

#include <Joystick.h>
#include "IMU/IMU.h"

class MotorControl
{
  #define MOTOR_CWF 2
  #define MOTOR_CWB 3
  #define MOTOR_CCWF 1
  #define MOTOR_CCWB 0

  #define JOYSTICK_CENTER 125
  #define JOYSTICK_THRESHOLD 5
  #define JOYSTICK_COMMAND_THRESHOLD 100
  #define DIR_DOWN JOYSTICK_CENTER - JOYSTICK_COMMAND_THRESHOLD
  #define DIR_UP JOYSTICK_CENTER + JOYSTICK_COMMAND_THRESHOLD
  #define DIR_LEFT JOYSTICK_CENTER + JOYSTICK_COMMAND_THRESHOLD
  #define DIR_RIGHT JOYSTICK_CENTER - JOYSTICK_COMMAND_THRESHOLD
  #define PID_INCREMENT 0.1
  #define ACCL_RATIO_INCREMENT 0.01

  // low pass ratio for ComplementaryFilter
  #define DEFAULT_ACCL_RATIO 0.02

  #define JOYSTICK_MIN_THROTTLE 16
  #define JOYSTICK_PITCH_LIMIT 10
  #define JOYSTICK_ROLL_LIMIT 10
  #define YAW_LIMIT 20

  #define PWM_FREQ 1000
  #define PWM_PERIOD_MS (1.0f / PWM_FREQ * 1000)

	public:
		void Init(double kp, double ki, double kd);
    void HandleJoystick(JoystickState state);
    void CalculateTargetAngles(struct ACCL_T *accl, struct GYRO_T *gyro);
    void SetMotorSpeed(float m1, float m2, float m3, float m4);
    void SetGyroOffset(float x, float y, float z);
    void DebugOrientation();

	private:
    void HandleJoystickCommands(JoystickState* state);
    void BlinkLed(int blinkTime, int count);
    int WaitForHoldCommand(unsigned long* start, int holdTime);
    void ComplementaryFilter(struct ACCL_T *accl, struct GYRO_T *gyro, float deltaTime);
    void AcclAngle(struct ACCL_T *accl);
    void UpdateMotorSpeed();
    int Clamp(int val, int min, int max);
    void HandleCommand(JoystickState* state);
    void PollCommand(JoystickState* state);
};

#endif
