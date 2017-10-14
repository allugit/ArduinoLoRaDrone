#ifndef MotorControl_h
#define MotorControl_h

#include <Arduino.h>
#include <Servo.h>
#include <PID_v1.h>

#include <Joystick.h>
#include "../IMU/IMU.h"

class MotorControl
{
  #define MOTOR_CWF 10
  #define MOTOR_CWB 11
  #define MOTOR_CCWF 12
  #define MOTOR_CCWB 9

  #define JOYSTICK_CENTER 125
  #define JOYSTICK_THRESHOLD 3
  #define JOYSTICK_COMMAND_THRESHOLD 30
  #define DIR_DOWN JOYSTICK_CENTER - JOYSTICK_COMMAND_THRESHOLD
  #define DIR_UP JOYSTICK_CENTER + JOYSTICK_COMMAND_THRESHOLD
  #define DIR_LEFT JOYSTICK_CENTER - JOYSTICK_COMMAND_THRESHOLD
  #define DIR_RIGHT JOYSTICK_CENTER + JOYSTICK_COMMAND_THRESHOLD
  #define PID_INCREMENT 1

  // low pass ratio for ComplementaryFilter
  #define LOW_PASS_RATIO 0.05
  #define PITCH_LIMIT 30
  #define ROLL_LIMIT 30
  #define YAW_LIMIT 20

	public:
		void Init(double kp, double ki, double kd);
    void HandleJoystick(JoystickState state);
    void CalculateTargetAngles(struct ACCL_T *accl, struct GYRO_T *gyro);
    void SetMotorSpeed(float m1, float m2, float m3, float m4);

	private:
    void HandleJoystickCommands(JoystickState* state);
    int WaitForHoldCommand(unsigned long* start, int holdTime);
    void ComplementaryFilter(struct ACCL_T *accl, struct GYRO_T *gyro, float deltaTime);
    void AcclAngle(struct ACCL_T *accl);
    void LimitAngles(double *pitch, double *roll);
    void UpdateMotorSpeed();
    int Clamp(int val, int min, int max);
};

#endif
