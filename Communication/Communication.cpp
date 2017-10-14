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

unsigned char* Communication::ReceiveRadioPacket()
{
	if (rf95.available())
	{
		unsigned char* buf = (unsigned char*)malloc(PAYLOAD_LENGTH);
		unsigned char len = PAYLOAD_LENGTH;

		if (rf95.recv(buf, &len))
		{
			// RH_RF95::printBuffer("Received: ", buf, len);
			// Serial.print("RSSI: ");
			// Serial.println(rf95.lastRssi(), DEC);
			//Serial.println("Receive");
			if (len == PAYLOAD_LENGTH && ValidatePacket(buf)) {
				return buf;
			}
		}
		else
		{
			//Serial.println("Receive failed");
		}
	}
	else {
		//Serial.println("Not available");
	}

	return NULL;
}

void Communication::SendRadioPacket(unsigned char* packet)
{
	CalculateCRC(packet, PAYLOAD_LENGTH);
	XorData(packet, PAYLOAD_LENGTH);

	rf95.send(packet, PAYLOAD_LENGTH);
	rf95.waitPacketSent();
}

int Communication::ValidatePacket(unsigned char* packet)
{
	XorData(packet, PAYLOAD_LENGTH);
	int crc = packet[CRC_POSITION];

	CalculateCRC(packet, PAYLOAD_LENGTH);

	if (crc == packet[CRC_POSITION]) {
		return 1;
	}

	return 0;
}

void Communication::CalculateCRC(unsigned char* data, unsigned char length)
{
	unsigned char i, crc;

	// CRC is the last byte, won't affect calculation when set to 0
	data[CRC_POSITION] = 0;

	for (i = 0, crc = 0; i < length; i++) {
		crc += data[i];
	}

	data[CRC_POSITION] = crc;
}

void Communication::XorData(unsigned char* data, unsigned char length)
{
	unsigned char i;

	for (i = 0; i < length; i++) {
		data[i] ^= XOR_KEY[i % KEY_LEN];
	}
}
