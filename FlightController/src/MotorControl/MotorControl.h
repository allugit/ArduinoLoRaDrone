#ifndef MotorControl_h
#define MotorControl_h

class MotorControl
{
  #define MOTOR_CWF 10
  #define MOTOR_CWB 11
  #define MOTOR_CCWF 12
  #define MOTOR_CCWB 9

  #define JOYSTICK_CENTER 125
  #define JOYSTICK_THRESHOLD 3
  #define PITCH_LIMIT 30
  #define ROLL_LIMIT 30

  // low pass ratio for ComplementaryFilter
  #define LOW_PASS_RATIO 0.05

	public:
		void Init(double kp, double ki, double kd);
    void HandleJoystick(unsigned char rx, unsigned char ry, unsigned char lx, unsigned char ly, unsigned char rz);
    void CalculateTargetAngles(struct ACCL_T *accl, struct GYRO_T *gyro);
    void SetMotorSpeed(float m1, float m2, float m3, float m4);

	private:
    void ComplementaryFilter(struct ACCL_T *accl, struct GYRO_T *gyro, float deltaTime);
    void AcclAngle(struct ACCL_T *accl);
    void LimitAngles(double *pitch, double *roll);
    void UpdateMotorSpeed();
    int Clamp(int val, int min, int max);
};

#endif
