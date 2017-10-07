#include "Arduino.h"
#include "ArduinoAdapter.h"

void ArduinoAdapter::setDigitalPortMode(AdapterPort port, char mode)
{
	int* port_pins = getPortPins(port);

	for (int i = 0; i < PIN_COUNT; i++)
		pinMode(port_pins[i], (mode >> i) & 1);
}


void ArduinoAdapter::writeDigitalPort(AdapterPort port, char value)
{
	int* port_pins = getPortPins(port);

	for (int i = 0; i < PIN_COUNT; i++)
		digitalWrite(port_pins[i], (value >> i) & 1);
}


char ArduinoAdapter::readDigitalPort(AdapterPort port)
{
	int* port_pins = getPortPins(port);
	char value = 0;

	// iterate each pin and shift value according to bit significance
	for (int i = 0; i < PIN_COUNT; i++)
		value += (digitalRead(port_pins[i]) << i);

	return value;
}


int* ArduinoAdapter::getPortPins(AdapterPort port)
{
	if (port == AdapterPort::B)
		return port_b;
	else if (port == AdapterPort::C)
		return port_c;
	else if (port == AdapterPort::D)
		return port_d;

	return {};
}
