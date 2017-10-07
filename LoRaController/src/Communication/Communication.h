#ifndef Communication_h
#define Communication_h

#include <Arduino.h>
#include <RH_RF95.h>

  // #define RFM95_CS 46
  // #define RFM95_RST 45
  // #define RFM95_INT 2
  #define RFM95_CS 8
  #define RFM95_RST 4
  #define RFM95_INT 7
  #define RF95_FREQ 868.1

  #define PAYLOAD_LENGTH 7
  #define CRC_POSITION 6
  #define KEY_LEN 3

class Communication
{
	const char XOR_KEY[KEY_LEN] = { 1011001, 0011010, 10000001 };

	public:
		void Init();
		char* ReceiveRadioPacket();
		void SendRadioPacket(char* packet);

	private:
		int ValidatePacket(char* packet);
		void CalculateCRC(char* data, char length);
		void XorData(char* data, char length);
};

#endif
