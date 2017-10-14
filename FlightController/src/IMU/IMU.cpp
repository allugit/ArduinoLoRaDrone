#include "IMU.h"

static SPISettings spiSettings(CLK_SPEED, MSBFIRST, SPI_MODE0);

void IMU::Init()
{
  SPI.begin(); //done by RadioHead for LoRa

  // chip select pins
  pinMode(INST_AG, OUTPUT);
  pinMode(INST_MAG, OUTPUT);
  pinMode(INST_ALT, OUTPUT);
  digitalWrite(INST_AG, HIGH);
  digitalWrite(INST_MAG, HIGH);
  digitalWrite(INST_ALT, HIGH);

  // init sensors on IMU
  InitAG(1);
  InitMAG(1);
  InitALT(1);

  // the startup range for the accelerometer is +/- 2g, which corresponds to a LSB value of 0.061mg/LSB
  m_GRangeLSB = GetXLRangeLSB(PAR_XL_2G);
  // the startup range for the gyro is +/- 245dps, which corresponds to a LSB value of 8.75mdps/LSB
  m_DPSRangeLSB = GetGRangeLSB(PAR_G_245DPS);
  // the startup range for the magnetometer is +/- 4Gauss, which corresponds to a LSB value of 0.14mGauss/LSB
  m_GaussRangeLSB = GetMAGRangeLSB(PAR_MAG_4GAUSS);
  //reference pressure corresponding to sea level
  Pref = 1013.25;
}

void IMU::GetDeviceID(struct DEV_ID *id)
{
	ReadRegister(INST_AG, WHO_AM_I, 1, &id->ag);
  ReadRegister(INST_MAG, WHO_AM_I, 1, &id->mag);
	ReadRegister(INST_ALT, WHO_AM_I, 1, &id->alt);
}

/*
**		This function provides the 3 "raw" 16-bit values read from the gyro.
**			-	It reads simultaneously the gyro value on three axes in a buffer of 6 bytes using the ReadRegister function
**			-	For each of the three axes, combines the two bytes in order to get a 16-bit value
*/
void IMU::ReadGyro(struct GYRO_T *gyro)
{
	uint8_t iGX_L, iGX_H, iGY_L, iGY_H, iGZ_L, iGZ_H;
	uint8_t rgwRegVals[6];

	//reads the bytes using the incremeting address functionality of the device.
	ReadRegister(INST_AG, OUT_X_L_G, 6, (uint8_t *)rgwRegVals);
	iGX_L = rgwRegVals[0];
	iGX_H = rgwRegVals[1];
	iGY_L = rgwRegVals[2];
	iGY_H = rgwRegVals[3];
	iGZ_L = rgwRegVals[4];
	iGZ_H = rgwRegVals[5];

	//combines the read values for each axis to obtain the 16-bits values
	int16_t rawGX = ((int16_t)iGX_H << 8) | iGX_L;
	int16_t rawGY = ((int16_t)iGY_H << 8) | iGY_L;
	int16_t rawGZ = ((int16_t)iGZ_H << 8) | iGZ_L;

  gyro->X = (float)rawGX * m_DPSRangeLSB;
  gyro->Y = (float)rawGY * m_DPSRangeLSB;
  gyro->Z = (float)rawGZ * m_DPSRangeLSB;
}



/*
**		provides the 3 "raw" 16-bit values read from the accelerometer.
**			-	It reads simultaneously the acceleration on three axes in a buffer of 6 bytes using the ReadRegister function
**			-	For each of the three axes, it combines the two bytes in order to get a 16-bit value
*/
void IMU::ReadAccel(struct ACCL_T *accl)
{
	uint8_t iAclX_L, iAclX_H, iAclY_L, iAclY_H, iAclZ_L, iAclZ_H;
	uint8_t rgwRegVals[6];

	//reads the bytes using the incremeting address functionality of the device.
	ReadRegister(INST_AG, OUT_X_L_XL, 6, (uint8_t *)rgwRegVals);
	iAclX_L = rgwRegVals[0];
	iAclX_H = rgwRegVals[1];
	iAclY_L = rgwRegVals[2];
	iAclY_H = rgwRegVals[3];
	iAclZ_L = rgwRegVals[4];
	iAclZ_H = rgwRegVals[5];

	// combines the read values for each axis to obtain the 16-bit values
  // and convert the accelerometer value to G's.
  int16_t rawAclX = ((int16_t)iAclX_H << 8) | iAclX_L;
	int16_t rawAclY = ((int16_t)iAclY_H << 8) | iAclY_L;
	int16_t rawAclZ = ((int16_t)iAclZ_H << 8) | iAclZ_L;

	accl->X = (float)rawAclX * m_GRangeLSB;
	accl->Y = (float)rawAclY * m_GRangeLSB;
	accl->Z = (float)rawAclZ * m_GRangeLSB;
}


// pressure in hPa
float IMU::ReadPressurehPa()
{
  uint8_t iPress_XL, iPress_L, iPress_H;
  uint8_t rgwRegVals[3];
  int32_t dataPress;

  ReadRegister(INST_ALT, PRESS_OUT_XL, 3, (uint8_t *)rgwRegVals);

  iPress_XL = rgwRegVals[0];
  iPress_L = rgwRegVals[1];
  iPress_H = rgwRegVals[2];
  dataPress = (iPress_H << 16) | (iPress_L << 8) | iPress_XL;

	//check if there is a negative value
	if (dataPress & 0x00800000) {
    dataPress |= 0xFF000000;
  }

  // to hPascals
	float hPa = dataPress / 4096;
	return hPa;
}

/*
**		This function calls the ConvPresToAltF function to obtain the altitude in feet. Then it converts it in meters
**		The Pref is computed once and used for further calculations of the altitude.
*/
float IMU::ConvPresToAltM(float hPa)
{
  float altFeet = (1 - pow(hPa / Pref, 0.190284)) * 145366.45;
	float altMeters = altFeet * 0.3048;

	return altMeters;
}

/*
**		The formula used is taken from the same technical note as for pressure.
**		T = 42.5 + TempRaw *1/480, where 480 LSB/�C represents the sensitivity of the temperature.
*/
float IMU::ReadTempC()
{
	uint8_t rgwRegVals[2], tempL, tempH;
	ReadRegister(INST_ALT, TEMP_OUT_L, 2, (uint8_t *)rgwRegVals);

	tempL = rgwRegVals[0];
	tempH = rgwRegVals[1];
	int16_t temp = (int16_t)tempH << 8 | tempL;

	//datasheet formula used for converting to temperature in degrees Celsius from raw values
	float tempC = 42.5 + (temp * 0.002083);
	return tempC;
}

/*
**		bAddr			- register address to start reading bytes from
**		bCntBytes		- number of bytes to be read
**		pData			- pointer to the 16 bit data array to be read
**		Reads bCntBytes bytes from device via SPI, from register having consecutive addresses, starting with bAddr.
*/
void IMU::ReadRegister(uint8_t chipSelect, uint8_t bAddr, uint8_t bCntBytes, uint8_t *pData)
{
	int ib;

  // only disable LoRa interrupt and allow PWM interrupt to interrupt this
  EIMSK &= (0 << INT6);
  digitalWrite(chipSelect, LOW);

  // send first byte indicating the operation will be reading from SPI
  SPI.transfer(bAddr | (chipSelect == INST_AG ? 0x80 : 0xC0));

  for(ib = 0; ib < bCntBytes; ib++)
  {
    // read from SPI the number of bytes given by bCntBytes
    uint8_t byte = SPI.transfer(0x00);
    pData[ib] = byte;
  }

  digitalWrite(chipSelect, HIGH);
  EIMSK |= (1 << INT6);
}

/*
**		bInst			- instrument Chip Select: Accelerometer/Gyroscope, Magnetometer or Altimeter selection
**		bAddr			- address to write to via SPI
**		bVal			- byte to be written to SPI
*/
void IMU::WriteSPI(uint8_t chipSelect, uint8_t bAddr, uint8_t bVal)
{
  // only disable LoRa interrupt and allow PWM interrupt to interrupt this
  EIMSK &= (0 << INT6);
  digitalWrite(chipSelect, LOW);

  //write first byte indicating the operation will be writing
  SPI.transfer(bAddr | 0x00);
  SPI.transfer(bVal);

  digitalWrite(chipSelect, HIGH);
  EIMSK |= (1 << INT6);
}

/* ------------------------------------------------------------ */
/*
**
**  bool fInit - init or shutdown
**
**	Description:
**		Initializes the accelerometer only or both the accelerometer and gyroscope instruments with the following settings:
**			for ACL mode:
**				�	Enable all three axes in CTRL_REG5_XL register,
**				�	set 10 Hz ODR in CTRL_REG6_XL register
**			for ACL+GYRO:
**				�	set 10Hz ODR in CTRL_REG1_G register and CTRL_REG6_XL, thus enabling the Gyroscope functionality as well;
**				�	enable the output of all three axes.
*/
void IMU::InitAG(bool fInit)
{
	if (fInit)
	{
  	//enable all three axes
  	WriteSPI(INST_AG, CTRL_REG5_XL,0x38);
    // 0x00 = -2g...2g, 0x03 = -8g...8g
    WriteSPI(INST_AG, CTRL_REG6_XL, PAR_XL_2G);
  	//set 10Hz odr for accel when used together with gyro
    // writing to CTRL_REG1_G (10h) both
    // accelerometer and gyroscope are activated at the same ODR.
    //WriteSPI(INST_AG, CTRL_REG6_XL, 0x60);

  	//10 Hz = 0x20, 119 Hz = 0x60
  	WriteSPI(INST_AG, CTRL_REG1_G,0x80);
  	//enable the axes outputs for Gyro
  	WriteSPI(INST_AG, CTRL_REG4,0x38);
	}
	else
	{
		//power down both the accel and gyro instruments
		WriteSPI(INST_AG, CTRL_REG5_XL, 0x00);
		WriteSPI(INST_AG, CTRL_REG6_XL, 0x00);
		WriteSPI(INST_AG, CTRL_REG9, 0x00);

		WriteSPI(INST_AG, CTRL_REG4, 0x00);
		WriteSPI(INST_AG, CTRL_REG1_G, 0x00);
	}
}

/*
**		Initializes the magnetometer instrument with the following settings:
**			�	set medium performance mode
**			�	10H ODR, in register CTRL_REG1_M
**			�	disable I2C and enable SPI read and write operations,
**			�	set the operating mode to continuous in CTRL_REG3_M register.
*/
void IMU::InitMAG(bool fInit)
{
	if (fInit)
	{
		//set medium performance mode for x and y and 10Hz ODR for MAG,
		WriteSPI(INST_MAG, CTRL_REG1_M, 0x30);
		//set scale to +-4Gauss
		WriteSPI(INST_MAG, CTRL_REG2_M, 0);
		//disable I2C and enable SPI read and write operations,
		//set the operating mode to continuous conversion
		WriteSPI(INST_MAG, CTRL_REG3_M, 0x00);
		//set medium performance mode for z axis
		WriteSPI(INST_MAG, CTRL_REG4_M, 0x04);
		//cntinuous update of output registers
		WriteSPI(INST_MAG, CTRL_REG5_M, 0x00);
	}
	else
	{
		//power down the instrument
		WriteSPI(INST_MAG, CTRL_REG3_M,0x03);
	}
}

/*
**		Initializes the barometer/altimeter instrument with the following settings:
**			�	set active mode and 7Hz ODR rate, in register CTRL_REG1,
**			�	block data update active.
*/
void IMU::InitALT(bool fInit)
{
	uint8_t status;
	if (fInit)
	{
		//clean start
		WriteSPI(INST_ALT, CTRL_REG1, 0x00);
		delay(1);
		//set active the device and ODR to 7Hz
		WriteSPI(INST_ALT, CTRL_REG1, 0xA4);
		//increment address during multiple byte access disabled
		WriteSPI(INST_ALT, CTRL_REG2, 0x00);
		//no modification to interrupt sources
		WriteSPI(INST_ALT, CTRL_REG4, 0x00);
	}
	else
	{
		//power down the instrument
		WriteSPI(INST_ALT, CTRL_REG1,0x00);
	}
}

/*
**		bRangeXL	- the parameter specifying the g range. Can be one of the parameters from the following list:
**					0	PAR_XL_2G	Parameter g range : +/- 2g
**					1	PAR_XL_4G	Parameter g range : +/- 4g
**					2	PAR_XL_8G	Parameter g range : +/- 8g
**					3	PAR_XL_16G 	Parameter g range : +/- 16g
**
**
**  Description:
**		The function computes the range LSB based on the set range parameter. The argument values are between 0 and 3.
**		If the argument is within the accepted values range, it selects the range LSB value for further computations of the "g" value.
**		If value is outside this range, the default value is set.
*/
float IMU::GetXLRangeLSB(uint8_t bRangeXL)
{
	switch(bRangeXL)
	{
		case PAR_XL_2G: return 0.000061;
		case PAR_XL_4G: return 0.000122;
		case PAR_XL_8G: return 0.000244;
		case PAR_XL_16G: return 0.000732;
		default: return 0.000061;
	}
}


/*
**		bRangeG	- the parameter specifying the dps range. Can be one of the parameters from the following list:
**					0	PAR_G_245DPS	Parameter dps range : +/- 245dps
**					1	PAR_G_500DPS	Parameter dps range : +/- 500dps
**					3	PAR_G_2kDPS		Parameter dps range : +/- 2kdps
**
**   Description:
**		The function computes the range LSB based on the set range parameter. The accepted argument values are between 0 and 3.
**		If the argument is within the accepted values range, it selects the range LSB value for further computations of the "dps" value.
**		If value is outside this range, the default value is set.
**
*/
float IMU::GetGRangeLSB(uint8_t bRangeG)
{
	switch(bRangeG)
	{
		case PAR_G_245DPS: return 0.00875;
		case PAR_G_500DPS: return 0.0175;
		case PAR_G_2kDPS: return 0.07;
		default: return 0.00875;
	}
}


/*
**		uint8_t bRangeMAG	- the parameter specifying the gauss range. Can be one of the parameters from the following list:
**					0	PAR_MAG_4GAUSS	Parameter gauss range : +/- 4gauss
**					1	PAR_MAG_8GAUSS	Parameter gauss range : +/- 8gauss
**					2	PAR_MAG_12GAUSS	Parameter gauss range : +/- 12gauss
**					3	PAR_MAG_16GAUSS Parameter gauss range : +/- 16gauss
**  Description:
**		The function computes the range LSB based on the bRangeMAG parameter. The accepted argument values are between 0 and 3.
**		If the argument is within the accepted values range, it selects the range LSB value for further computations of the "gauss" value.
**		If value is outside this range, the default value is set.
**
*/
float IMU::GetMAGRangeLSB(uint8_t bRangeMAG)
{
	switch(bRangeMAG)
	{
		case PAR_MAG_4GAUSS: return 0.00014;
		case PAR_MAG_8GAUSS: return 0.00029;
		case PAR_MAG_12GAUSS: return 0.00043;
		case PAR_MAG_16GAUSS: return 0.00058;
		default: return 0.00014;
	}
}
