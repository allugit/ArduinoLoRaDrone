#ifndef Joystick_h
#define Joystick_h

#include <Arduino.h>

#define JOYSTICK_LX A0
#define JOYSTICK_LY A1
#define JOYSTICK_LZ 12

#define JOYSTICK_RX A2
#define JOYSTICK_RY A3
#define JOYSTICK_RZ 11

#define JOYSTICK_ADC_CENTER 512
#define JOYSTICK_CENTER_THRESHOLD 60

struct JoystickState {
	unsigned char LX, LY, LZ;
	unsigned char RX, RY, RZ;
};

class Joystick
{
	public:
        void Init();
        JoystickState GetJoystickAxises();

    private:
        void DebugJoystick(JoystickState state);
        void ScaleAxisValues(int *x, int *y, int *z);
        bool IsCenter(int axisValue);
        int Clamp(int value, int min, int max);
};

#endif
