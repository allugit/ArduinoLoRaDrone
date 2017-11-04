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

  #define ACT_THS             0B00000100
#define ACT_DUR             0B00000101
#define INT_GEN_CFG_XL      0B00000110
#define INT_GEN_THS_X_XL    0B00000111
#define INT_GEN_THS_Y_XL    0B00001000
#define INT_GEN_THS_Z_XL    0B00001001
#define INT_GEN_DUR_XL      0B00001010
#define REFERENCE_G         0B00001011
#define INT1_CTRL           0B00001100
#define INT2_CTRL           0B00001101

#define WHO_AM_I            0B00001111
#define CTRL_REG1_G         0B00010000
#define CTRL_REG2_G         0B00010001
#define CTRL_REG3_G         0B00010010
#define ORIENT_CFG_G        0B00010011
#define INT_GEN_SRC_G       0B00010100
#define OUT_TEMP_L          0B00010101
#define OUT_TEMP_H          0B00010110
#define STATUS_REG0         0B00010111
#define OUT_X_L_G           0B00011000
#define OUT_X_H_G           0B00011001
#define OUT_Y_L_G           0B00011010
#define OUT_Y_H_G           0B00011011
#define OUT_Z_L_G           0B00011100
#define OUT_Z_H_G           0B00011101
#define CTRL_REG4           0B00011110
#define CTRL_REG5_XL        0B00011111
#define CTRL_REG6_XL        0B00100000
#define CTRL_REG7_XL        0B00100001
#define CTRL_REG8           0B00100010
#define CTRL_REG9           0B00100011
#define CTRL_REG10          0B00100100

#define INT_GEN_SRC_XL      0B00100110
#define STATUS_REG          0B00100111
#define OUT_X_L_XL          0B00101000
#define OUT_X_H_XL          0B00101001
#define OUT_Y_L_XL          0B00101010
#define OUT_Y_H_XL          0B00101011
#define OUT_Z_L_XL          0B00101100
#define OUT_Z_H_XL          0B00101101
#define FIFO_CTRL           0B00101110
#define FIFO_SRC            0B00101111
#define INT_GEN_CFG_G       0B00110000
#define INT_GEN_THS_XH_G    0B00110001
#define INT_GEN_THS_XL_G    0B00110010
#define INT_GEN_THS_YH_G    0B00110011
#define INT_GEN_THS_YL_G    0B00110100
#define INT_GEN_THS_ZH_G    0B00110101
#define INT_GEN_THS_ZL_G    0B00110110
#define INT_GEN_DUR_G       0B00110111

#define OFFSET_X_REG_L_M    0B00000101
#define OFFSET_X_REG_H_M    0B00000110
#define OFFSET_Y_REG_L_M    0B00000111
#define OFFSET_Y_REG_H_M    0B00001000
#define OFFSET_Z_REG_L_M    0B00001001
#define OFFSET_Z_REG_H_M    0B00001010

#define WHO_AM_I_M          0B00001111

#define CTRL_REG1_M         0B00100000
#define CTRL_REG2_M         0B00100001
#define CTRL_REG3_M         0B00100010
#define CTRL_REG4_M         0B00100011
#define CTRL_REG5_M         0B00100100

#define STATUS_REG_M        0B00100111
#define OUT_X_L_M           0B00101000
#define OUT_X_H_M           0B00101001
#define OUT_Y_L_M           0B00101010
#define OUT_Y_H_M           0B00101011
#define OUT_Z_L_M           0B00101100
#define OUT_Z_H_M           0B00101101

#define INT_CFG_M           0B00110000
#define INT_SRC_M           0B00110001
#define INT_THS_L_M         0B00110010
#define INT_THS_H_M         0B00110011

#define REF_P_XL			0B00001000
#define REF_P_L				0B00001001
#define REF_P_H				0B00001010
#define WHO_AM_I			0B00001111
#define RES_CONF			0B00010000

#define CTRL_REG1			0B00100000
#define CTRL_REG2			0B00100001
#define CTRL_REG3			0B00100010
#define INTERRUPT_CFG		0B00100100
#define INT_SOURCE			0B00100101

#define STATUS_REG			0B00100111
#define PRESS_OUT_XL		0B00101000
#define PRESS_OUT_L			0B00101001
#define PRESS_OUT_H			0B00101010
#define TEMP_OUT_L			0B00101011
#define TEMP_OUT_H			0B00101100

#define FIFO_CTRL			0B00101110
#define FIFO_STATUS			0B00101111
#define THS_P_L				0B00110000
#define THS_P_H				0B00110001

#define RPDS_L				0B00111001
#define RPDS_H				0B00111010

#define DRDY_M			13 // valid for CerebotMX4cK board only
#define INT				12

#define MODE_INST_A		0
#define MODE_INST_AG	1
#define MODE_INST_MAG	2
#define MODE_INST_ALT	3

#define INT_PIN_1		0
#define	INT_PIN_2		1

#define	PAR_XL_2G		0
#define	PAR_XL_4G		2
#define	PAR_XL_8G		3
#define	PAR_XL_16G		1
#define	PAR_G_245DPS	0
#define	PAR_G_500DPS	1
#define	PAR_G_2kDPS		3
#define PAR_MAG_4GAUSS	0
#define PAR_MAG_8GAUSS	1
#define PAR_MAG_12GAUSS	2
#define PAR_MAG_16GAUSS	3

#define	PAR_INT_ACTIVEHIGH		0
#define	PAR_INT_ACTIVELOW		1
#define PAR_INT_PUSHPULL		0
#define PAR_INT_OPENDRAIN		1
#define	PAR_EXT_INT0			0
#define	PAR_EXT_INT1			1
#define	PAR_EXT_INT2			2
#define	PAR_EXT_INT3			3
#define	PAR_EXT_INT4			4

#define	FIFO_MODE_XL_G_BYPASS			0
#define	FIFO_MODE_XL_G_FIFO				1
#define	FIFO_MODE_XL_G_CONTINUOUS_FIFO	3
#define	FIFO_MODE_XL_G_BYPASS_CONTINUOUS		4
#define	FIFO_MODE_XL_G_CONTINUOUS		6

#define	FIFO_MODE_ALT_BYPASS			0
#define	FIFO_MODE_ALT_FIFO				1
#define	FIFO_MODE_ALT_STREAM			2
#define	FIFO_MODE_ALT_STREAM_TO_FIFO	3
#define	FIFO_MODE_ALT_BYPASS_TO_STREAM	4
#define	FIFO_MODE_ALT_MEAN				6
#define	FIFO_MODE_ALT_BYPASS_TO_FIFO	7

#define	FIFO_MODE_MEAN_ALT_2SAMPLES		0B00001
#define	FIFO_MODE_MEAN_ALT_4SAMPLES		0B00011
#define	FIFO_MODE_MEAN_ALT_8SAMPLES		0B00111
#define	FIFO_MODE_MEAN_ALT_16SAMPLES	0B01111
#define	FIFO_MODE_MEAN_ALT_32SAMPLES	0B11111

#define	ACL_NO_BITS		16
#define	X_AXIS			0
#define	Y_AXIS			1
#define	Z_AXIS			2
#define	ALL_AXIS		3
#define PI				3.1415

#define	MSK_RANGE_XL	0x18
#define	MSK_RANGE_G		0x18
#define MSK_RANGE_MAG	0x60
#define MSK_ODR_XL		0xE0
#define MSK_ODR_G		0xE0
#define MSK_ODR_MAG		0x1C
#define MSK_ODR_ALT		0x70
#define MSK_FIFO_CTL_MODE	0xE0
#define MSK_FIFO_THS	0x1F

#define	MSK_INT1_IG_G				1<<7
#define	MSK_INT1_IG_XL				1<<6
#define	MSK_INT1_FSS5				1<<5
#define	MSK_INT1_OVR				1<<4
#define	MSK_INT1_FTH				1<<3
#define	MSK_INT1_Boot				1<<2
#define	MSK_INT1_DRDY_G				1<<1
#define	MSK_INT1_DRDY_XL			1<<0

#define	MSK_INT2_INACT				1<<7
#define	MSK_INT2_FSS5				1<<5
#define	MSK_INT2_OVR				1<<4
#define	MSK_INT2_FTH				1<<3
#define	MSK_INT2_DRDY_TEMP			1<<2
#define	MSK_INT2_DRDY_G				1<<1
#define	MSK_INT2_DRDY_XL			1<<0

#define MSK_XLIE_XL 				1<<0
#define MSK_XHIE_XL 				1<<1
#define MSK_YLIE_XL 				1<<2
#define MSK_YHIE_XL 				1<<3
#define MSK_ZLIE_XL 				1<<4
#define MSK_ZHIE_XL 				1<<5
#define MSK_GEN_6D 					1<<6

#define MSK_XLIE_G	 				1<<0
#define MSK_XHIE_G					1<<1
#define MSK_YLIE_G					1<<2
#define MSK_YHIE_G					1<<3
#define MSK_ZLIE_G					1<<4
#define MSK_ZHIE_G					1<<5

#define MSK_ZIEN_MAG				1<<5
#define MSK_YIEN_MAG				1<<6
#define MSK_XIEN_MAG 				1<<7

#define MSK_DIFF_EN_ALT 			1<<3

#define MSK_INT_P_HIGH				0x01
#define MSK_INT_P_LOW				0x02
#define MSK_INT_P_LOW_HIGH			0x03
//int configurations for Altimeter
#define MSK_INT_F_EMPTY				1<<3
#define MSK_INT_F_FTH				1<<2
#define MSK_INT_F_OVR				1<<1
#define MSK_INT_DRDY				1<<0

#define MSK_INT_LEVEL_HIGH			1<<0
#define MSK_INT_LEVEL_LOW			1<<1

#define ODR_G_PWR_DWN	0
#define ODR_G_14_9HZ	1
#define ODR_G_59_5HZ	2
#define ODR_G_119HZ		3
#define ODR_G_238HZ		4
#define ODR_G_476HZ		5
#define ODR_G_952HZ		6
#define ODR_G_NA		7
//ODR defined values for Accelerometer
#define ODR_XL_PWR_DWN	0
#define ODR_XL_10HZ		1
#define ODR_XL_50HZ		2
#define ODR_XL_119HZ	3
#define ODR_XL_238HZ	4
#define ODR_XL_476HZ	5
#define ODR_XL_952HZ	6
#define ODR_XL_NA		7
//ODR defined values for magnetometer
#define ODR_M_0_625HZ	0
#define ODR_M_1_25HZ	1
#define ODR_M_2_5HZ		2
#define ODR_M_5HZ		3
#define ODR_M_10HZ		4
#define ODR_M_20HZ		5
#define ODR_M_40HZ		6
#define ODR_M_80HZ		7

#define ODR_ALT_ONE_SHOT 0
#define ODR_ALT_1HZ		1
#define ODR_ALT_7HZ		2
#define ODR_ALT_12_5HZ	3
#define ODR_ALT_25HZ	4

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
    void ReadMagPolar(struct POLAR_T *polar);
    float ReadPressurehPa();
    float ConvPresToAltM(float hPa);
    float ReadTempC();

  private:
    void InitAG(bool fInit);
    void InitMAG(bool fInit);
    void InitALT(bool fInit);

    void ReadRegister(uint8_t chipSelect, uint8_t bAddr, uint8_t bCntBytes, uint8_t *pData);
    void WriteSPI(uint8_t chipSelect, uint8_t bAddr, uint8_t bVal);
    void WriteRegister(uint8_t bInst, uint8_t bAddr, uint8_t bCntBytes, uint8_t *pData);

    float GetMAGRangeLSB(uint8_t bRangeMAG);
    float GetGRangeLSB(uint8_t bRangeG);
    float GetXLRangeLSB(uint8_t bRangeXL);
    void SetRangeMAG(uint8_t bRangeMAG);
    uint8_t GetRangeMAG();

    void CalibrateMag(bool loadIn);
    void MagOffset(uint8_t axis, int16_t offset);
    uint8_t DataAvailableMAG(uint8_t axis);
    void ReadMag(int16_t &MagX, int16_t &MagY, int16_t &MagZ);
    float ConvertReadingToValueGauss(int16_t rawVal);
    void ConvMagToPolar(struct MAGNO_T *mag, struct POLAR_T *polar);

    uint8_t GetBitsInRegister(uint8_t bInst, uint8_t bRegAddr, uint8_t startBit, uint8_t noBits);
	  void SetRegisterBits(uint8_t bInst, uint8_t bRegAddr, uint8_t bMask, bool fValue);
	  void SetBitsInRegister(uint8_t bInst, uint8_t bRegAddr, uint8_t bMask, uint8_t bValue, uint8_t startBit);
};

#endif
