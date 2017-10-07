#include "Communication.h"

RH_RF95 rf95(RFM95_CS, RFM95_INT);
int receives;

void Communication::Init()
{
	pinMode(RFM95_RST, OUTPUT);
	digitalWrite(RFM95_RST, HIGH);
	delay(100);

	// manual reset
	digitalWrite(RFM95_RST, LOW);
	delay(1);
	digitalWrite(RFM95_RST, HIGH);
	delay(1);

	while (!rf95.init()) {
		while (1);
	}

	// Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
	if (!rf95.setFrequency(RF95_FREQ)) {
		while (1);
	}

	// Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on
	// The default transmitter power is 13dBm, using PA_BOOST.
	// If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then
	// you can set transmitter powers from 5 to 23 dBm:
	rf95.setTxPower(23, false);
}

char* Communication::ReceiveRadioPacket()
{
	if (rf95.available())
	{
		char buf[PAYLOAD_LENGTH];
		char len = sizeof(buf);

		if (rf95.recv(buf, &len))
		{
			// RH_RF95::printBuffer("Received: ", buf, len);
			// Serial.print("RSSI: ");
			// Serial.println(rf95.lastRssi(), DEC);

			if (ValidatePacket(buf)) {
				return buf;
			}

			return NULL;
		}
		//else
		//{
		//	Serial.println("Receive failed");
		//}
	}
}

void Communication::SendRadioPacket(char* packet)
{
	CalculateCRC(packet, PAYLOAD_LENGTH);
	XorData(packet, PAYLOAD_LENGTH);

	rf95.send(packet, PAYLOAD_LENGTH);
	rf95.waitPacketSent();
}

int Communication::ValidatePacket(char* packet)
{
	XorData(packet, PAYLOAD_LENGTH);
	int crc = packet[CRC_POSITION];

	CalculateCRC(packet, PAYLOAD_LENGTH);

	if (crc == packet[CRC_POSITION]) {
		return 1;
	}

	return 0;
}

void Communication::CalculateCRC(char* data, char length)
{
	char i, crc;

	// CRC is the last byte, won't affect calculation when set to 0
	data[CRC_POSITION] = 0;

	for (i = 0, crc = 0; i < length; i++) {
		crc += data[i];
	}

	data[CRC_POSITION] = crc;
}

void Communication::XorData(char* data, char length)
{
	char i;

	for (i = 0; i < length; i++) {
		data[i] ^= XOR_KEY[i % KEY_LEN];
	}
}