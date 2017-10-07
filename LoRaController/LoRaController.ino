#include "src/Communication/Communication.h"
#include "src/Joystick/Joystick.h"
 
Communication comm;
Joystick joystick;
 
void setup() 
{
  comm.Init();
  joystick.Init();
}

void loop()
{

  JoystickState state = joystick.GetJoystickAxises();
  char packet[PAYLOAD_LENGTH];
  
  packet[0] = state.LX;
  packet[1] = state.LY;
  packet[2] = state.LZ;

  packet[3] = state.RX;
  packet[4] = state.RY;
  packet[5] = state.RZ;

  comm.SendRadioPacket(packet);
}
