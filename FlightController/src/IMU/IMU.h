#ifndef IMU_h
#define IMU_h

#include <Arduino.h>
#include <SPI.h>
#include <util/atomic.h>

// data structs measurement results
struct ACCL_T { float X; float Y; float Z; };
struct GYRO_T { float X; float Y; float Z; };
struct MAGNO_T { float X; float Y; float Z; };
struct POLAR_T { float R; float D; };
struct DEV_ID  { uint8_t ag; uint8_t mag; uint8_t alt; };

class IMU
{
  // Chip select
  #define INST_ALT 22
  #define INST_MAG 21
  #define INST_AG 23

  // SPI clock speed
  #define CLK_SPEED 1000000 // 4000000

  // some registers addresses for IMU
  #define WHO_AM_I      0B00001111
  #define OUT_X_L_G     0B00011000
  #define OUT_X_L_XL    0B00101000
  #define PRESS_OUT_XL	0B00101000
  #define PRESS_OUT_L		0B00101001
  #define PRESS_OUT_H		0B00101010
  #define TEMP_OUT_L		0B00101011
  #define TEMP_OUT_H		0B00101100

  #define CTRL_REG1_M   0B00100000
  #define CTRL_REG2_M   0B00100001
  #define CTRL_REG3_M   0B00100010
  #define CTRL_REG4_M   0B00100011
  #define CTRL_REG5_M   0B00100100

  #define CTRL_REG1			0B00100000
  #define CTRL_REG2			0B00100001
  #define CTRL_REG3			0B00100010
  #define CTRL_REG4			0B00100011
  #define CTRL_REG5_XL  0B00011111
  #define CTRL_REG6_XL  0B00100000
  #define CTRL_REG7_XL  0B00100001
  #define CTRL_REG8     0B00100010
  #define CTRL_REG9     0B00100011
  #define CTRL_REG10    0B00100100

  #define CTRL_REG1_G   0B00010000
  #define CTRL_REG2_G   0B00010001
  #define CTRL_REG3_G   0B00010010

  // value conversion settings: accel, gyro, magnetometer
  #define	PAR_XL_2G		    0
  #define	PAR_XL_4G		    2
  #define	PAR_XL_8G		    3
  #define	PAR_XL_16G		  1
  #define	PAR_G_245DPS	  0
  #define	PAR_G_500DPS	  1
  #define	PAR_G_2kDPS		  3
  #define PAR_MAG_4GAUSS	0
  #define PAR_MAG_8GAUSS	1
  #define PAR_MAG_12GAUSS	2
  #define PAR_MAG_16GAUSS	3

  // "raw" value conversion params
  float m_GRangeLSB;
	float m_DPSRangeLSB;
	float m_GaussRangeLSB;

  // pressure reference
	float Pref;

  public:
    void Init();
    void GetDeviceID(struct DEV_ID *id);
    void ReadGyro(struct GYRO_T *gyro);
    void ReadAccel(struct ACCL_T *accl);
    float ReadPressurehPa();
    float ConvPresToAltM(float hPa);
    float ReadTempC();

  private:
    void InitAG(bool fInit);
    void InitMAG(bool fInit);
    void InitALT(bool fInit);

    void ReadRegister(uint8_t chipSelect, uint8_t bAddr, uint8_t bCntBytes, uint8_t *pData);
    void WriteSPI(uint8_t chipSelect, uint8_t bAddr, uint8_t bVal);

    float GetMAGRangeLSB(uint8_t bRangeMAG);
    float GetGRangeLSB(uint8_t bRangeG);
    float GetXLRangeLSB(uint8_t bRangeXL);
};

#endif
