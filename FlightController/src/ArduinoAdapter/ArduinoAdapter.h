#ifndef ArduinoAdapter_h
#define ArduinoAdapter_h

// Enum type for selecting a specific port on Arduino adapter
enum class AdapterPort { B, C, D };

class ArduinoAdapter
{
	#define PIN_COUNT 8

	public:
		// set port pin modes with an 8 bit value
		// 0xFF = B11111111 = all pins OUTPUT
		// 0x0F = B00001111 = pins 0-3 OUTPUT, 4-7 INTPUT
		void setDigitalPortMode(AdapterPort port, char mode);

		// write 8 bit value to OUTPUT port
		void writeDigitalPort(AdapterPort port, char value);

		// read INPUT port digital value
		char readDigitalPort(AdapterPort port);


	private:
		int* getPortPins(AdapterPort port);
		// -1 is a non available pin due to pin mappings
		int port_b[PIN_COUNT] = { 8, 9, 10, 11, 12, 13, -1, -1 };
		int port_c[PIN_COUNT] = { A0, A1, A2, A3, A4, A5, -1, -1 };
		int port_d[PIN_COUNT] = { 0, 1, 2, 3, 4, 5, 6, 7 };
};

#endif
