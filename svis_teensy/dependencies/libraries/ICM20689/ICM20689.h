#ifndef _ICM20689_H_
#define _ICM20689_H_

#include "I2Cdev.h"

#include <avr/pgmspace.h>

//ADDRESSES
#define ICM20689_ADDRESS_AD0_LOW     0x68 // address pin low (GND), default for InvenSense evaluation board
#define ICM20689_ADDRESS_AD0_HIGH    0x69 // address pin high (VCC)
#define ICM20689_DEFAULT_ADDRESS     ICM20689_ADDRESS_AD0_LOW

#define ICM20689_WHO_AM_I			0x98

//REGISTERS
#define ICM20689_RA_SELF_TEST_X_GYRO	0x00
#define ICM20689_RA_SELF_TEST_Y_GYRO	0x01
#define ICM20689_RA_SELF_TEST_Z_GYRO	0x02
#define ICM20689_RA_SELF_TEST_X_ACCEL	0x0D
#define ICM20689_RA_SELF_TEST_Y_ACCEL	0x0E
#define ICM20689_RA_SELF_TEST_Z_ACCEL	0x0F
#define ICM20689_RA_XG_OFFS_USRH		0x13
#define ICM20689_RA_XG_OFFS_USRL		0x14
#define ICM20689_RA_YG_OFFS_USRH		0x15
#define ICM20689_RA_YG_OFFS_USRL		0x16
#define ICM20689_RA_ZG_OFFS_USRH		0x17
#define ICM20689_RA_ZG_OFFS_USRL		0x18
#define ICM20689_RA_SMPLRT_DIV			0x19
#define ICM20689_RA_CONFIG				0x1A
#define ICM20689_RA_GYRO_CONFIG			0x1B
#define ICM20689_RA_ACCEL_CONFIG_1		0x1C
#define ICM20689_RA_ACCEL_CONFIG_2		0x1D
#define ICM20689_RA_LP_MODE_CFG			0x1E
#define ICM20689_RA_ACCEL_WOM_THR		0x1F
#define ICM20689_RA_FIFO_EN				0x23
#define ICM20689_RA_FSYNC_INT			0x36
#define ICM20689_RA_INT_PIN_CFG			0x37
#define ICM20689_RA_INT_ENABLE			0x38
#define ICM20689_RA_DMP_INT_STATUS		0x39
#define ICM20689_RA_INT_STATUS			0x3A
#define ICM20689_RA_ACCEL_XOUT_H		0x3B
#define ICM20689_RA_ACCEL_XOUT_L		0x3C
#define ICM20689_RA_ACCEL_YOUT_H		0x3D
#define ICM20689_RA_ACCEL_YOUT_L		0x3E
#define ICM20689_RA_ACCEL_ZOUT_H		0x3F
#define ICM20689_RA_ACCEL_ZOUT_L		0x40
#define ICM20689_RA_TEMP_OUT_H			0x41
#define ICM20689_RA_TEMP_OUT_L			0x42
#define ICM20689_RA_GYRO_XOUT_H			0x43
#define ICM20689_RA_GYRO_XOUT_L			0x44
#define ICM20689_RA_GYRO_YOUT_H			0x45
#define ICM20689_RA_GYRO_YOUT_L			0x46
#define ICM20689_RA_GYRO_ZOUT_H			0x47
#define ICM20689_RA_GYRO_ZOUT_L			0x48
#define ICM20689_RA_SIGNAL_PATH_RESET	0x68
#define ICM20689_RA_ACCEL_INTEL_CTRL	0x69
#define ICM20689_RA_USER_CTRL			0x6A
#define ICM20689_RA_PWR_MGMT_1			0x6B
#define ICM20689_RA_PWR_MGMT_2			0x6C
#define ICM20689_RA_FIFO_COUNTH			0x72
#define ICM20689_RA_FIFO_COUNTL			0x73
#define ICM20689_RA_FIFO_R_W			0x74
#define ICM20689_RA_WHO_AM_I			0x75
#define ICM20689_RA_XA_OFFSET_H			0x77
#define ICM20689_RA_XA_OFFSET_L			0x78
#define ICM20689_RA_YA_OFFSET_H			0x7A
#define ICM20689_RA_YA_OFFSET_L			0x7B
#define ICM20689_RA_ZA_OFFSET_H			0x7D
#define ICM20689_RA_ZA_OFFSET_L			0x7E

//CONFIG
#define ICM20689_CFG_FIFO_MODE_BIT			6
#define ICM20689_CFG_EXT_SYNC_SET_BIT    	5
#define ICM20689_CFG_EXT_SYNC_SET_LENGTH 	3
#define ICM20689_CFG_DLPF_CFG_BIT    		2
#define ICM20689_CFG_DLPF_CFG_LENGTH 		3

//GYRO CONFIG
#define ICM20689_GCONFIG_XG_ST_BIT       7
#define ICM20689_GCONFIG_YG_ST_BIT       6
#define ICM20689_GCONFIG_ZG_ST_BIT       5

#define ICM20689_GCONFIG_FS_SEL_BIT      4
#define ICM20689_GCONFIG_FS_SEL_LENGTH   2

#define ICM20689_GYRO_FS_250		0x00
#define ICM20689_GYRO_FS_500     	0x01
#define ICM20689_GYRO_FS_1000    	0x02
#define ICM20689_GYRO_FS_2000    	0x03

//ACCEL CONFIG 1
#define ICM20689_ACONFIG_XA_ST_BIT          7
#define ICM20689_ACONFIG_YA_ST_BIT          6
#define ICM20689_ACONFIG_ZA_ST_BIT          5

#define ICM20689_ACONFIG_AFS_SEL_BIT        4
#define ICM20689_ACONFIG_AFS_SEL_LENGTH     2

#define ICM20689_ACCEL_FS_2          0x00
#define ICM20689_ACCEL_FS_4          0x01
#define ICM20689_ACCEL_FS_8          0x02
#define ICM20689_ACCEL_FS_16         0x03

//ACCEL CONFIG 2
#define ICM20689_ACONFIG_DEC2_CFG_BIT 				5
#define ICM20689_ACONFIG_DEC2_CFG_LENGTH			2
#define ICM20689_ACONFIG_ACCEL_FCHOICE_B_SET_BIT	3
#define ICM20689_ACONFIG_A_DLPF_CFG_BIT				2
#define ICM20689_ACONFIG_A_DLPF_CFG_LENGTH			3

#define ICM20689_ACCEL_AVG_4	0x00
#define ICM20689_ACCEL_AVG_8	0x01
#define ICM20689_ACCEL_AVG_16	0x02
#define ICM20689_ACCEL_AVG_32	0x03

//LOW POWER MODE CONFIG
#define ICM20689_LPMCFG_GYRO_LPM_SET_BIT	7
#define ICM20689_LPMCFG_GYRO_AVG_CFG_BIT	6
#define ICM20689_LPMCFG_GYRO_AVG_CFG_LENGTH	3

//FSYNC INT STATUS
#define ICM20689_FSYNC_INT_STATUS	7

//FIFO
#define ICM20689_TEMP_FIFO_EN_BIT    7
#define ICM20689_XG_FIFO_EN_BIT      6
#define ICM20689_YG_FIFO_EN_BIT      5
#define ICM20689_ZG_FIFO_EN_BIT      4
#define ICM20689_ACCEL_FIFO_EN_BIT   3

//INT CONFIG
#define ICM20689_INTCFG_INT_LEVEL_BIT        7
#define ICM20689_INTCFG_INT_OPEN_BIT         6
#define ICM20689_INTCFG_LATCH_INT_EN_BIT     5
#define ICM20689_INTCFG_INT_RD_CLEAR_BIT     4
#define ICM20689_INTCFG_FSYNC_INT_LEVEL_BIT  3
#define ICM20689_INTCFG_FSYNC_INT_EN_BIT     2

//INT EN
#define	ICM20689_WOM_INT_EN_BIT			7
#define ICM20689_WOM_INT_EN_LENGTH		3
#define ICM20689_FIFO_OFLOW_EN_BIT		4
#define ICM20689_GDRIVE_INT_EN_BIT		2
#define ICM20689_DMP_INT_EN_BIT			1
#define ICM20689_DATA_RDY_INT_EN_BIT	0

#define ICM20689_WOM_ENABLE		0x03
#define ICM20689_WOM_DISABLE	0x00

//INT STATUS
#define	ICM20689_WOM_INT_STAT_BIT			7
#define ICM20689_WOM_INT_STAT_LENGTH		3
#define ICM20689_FIFO_OFLOW_STAT_BIT		4
#define ICM20689_GDRIVE_INT_STAT_BIT		2
#define ICM20689_DMP_INT_STAT_BIT			1
#define ICM20689_DATA_RDY_STAT_BIT		0

//SIGNAL PATH RST
#define ICM20689_PATHRESET_ACCEL_RESET_BIT	1
#define ICM20689_PATHRESET_TEMP_RESET_BIT	0

//ACCELEROMETER INTELLIGENCE CONTROL
#define ICM20689_ACCEL_INTEL_EN_BIT		7
#define ICM20689_ACCEL_INTEL_MODE_BIT	6

//USER CONTROL
#define ICM20689_USERCTRL_DMP_EN_BIT			7
#define ICM20689_USERCTRL_FIFO_EN_BIT		6
#define ICM20689_USERCTRL_I2C_IF_DIS_BIT		4
#define ICM20689_USERCTRL_DMP_RST_BIT		3
#define ICM20689_USERCTRL_FIFO_RESET_BIT		2
#define ICM20689_USERCTRL_SIG_COND_RESET_BIT	0

//POWER MGMT 1
#define ICM20689_PWR1_DEVICE_RESET_BIT		7
#define ICM20689_PWR1_SLEEP_SET_BIT			6
#define ICM20689_PWR1_ACCEL_CYCLE_SET_BIT	5
#define ICM20689_PWR1_GYRO_STANDBY_SET_BIT	4
#define ICM20689_PWR1_TEMP_DISABLE_BIT		3
#define ICM20689_PWR1_CLKSEL_BIT			2
#define ICM20689_PWR1_CLKSEL_LENGTH			3

#define ICM20689_INT_OSC	0x00
#define ICM20689_BEST_CLK	0x01
#define ICM20689_STOP_CLK	0x07

//POWER MGMT 2
#define ICM20689_PWR2_FIFO_LP_EN_BIT	7
#define ICM20689_PWR2_DMP_LP_DIS_BIT	6
#define ICM20689_PWR2_STBY_XA_BIT		5
#define ICM20689_PWR2_STBY_YA_BIT		4
#define ICM20689_PWR2_STBY_ZA_BIT		3
#define ICM20689_PWR2_STBY_XG_BIT		2
#define ICM20689_PWR2_STBY_YG_BIT		1
#define ICM20689_PWR2_STBY_ZG_BIT		0

class ICM20689 {
    public:
        ICM20689();
        ICM20689(uint8_t address);

        void initialize();
        bool testConnection();

        // SMPLRT_DIV register
        uint8_t getRate();
        void setRate(uint8_t rate);

        // CONFIG register
		//bool getFIFOMode();
		//void setFIFOMode(bool enabled);
        uint8_t getExternalFrameSync();
        void setExternalFrameSync(uint8_t sync);
        uint8_t getDLPFMode();
        void setDLPFMode(uint8_t bandwidth);

        // GYRO_CONFIG register
		//bool getGyroXSelfTest();
        //void setGyroXSelfTest(bool enabled);
        //bool getGyroYSelfTest();
        //void setGyroYSelfTest(bool enabled);
        //bool getGyroZSelfTest();
        //void setGyroZSelfTest(bool enabled);
        uint8_t getFullScaleGyroRange();
        void setFullScaleGyroRange(uint8_t range);

		// SELF_TEST registers
		//uint8_t getAccelXSelfTestFactoryTrim();
		//uint8_t getAccelYSelfTestFactoryTrim();
		//uint8_t getAccelZSelfTestFactoryTrim();

		//uint8_t getGyroXSelfTestFactoryTrim();
		//uint8_t getGyroYSelfTestFactoryTrim();
		//uint8_t getGyroZSelfTestFactoryTrim();
		
        // ACCEL_CONFIG1 register
        bool getAccelXSelfTest();
        void setAccelXSelfTest(bool enabled);
        bool getAccelYSelfTest();
        void setAccelYSelfTest(bool enabled);
        bool getAccelZSelfTest();
        void setAccelZSelfTest(bool enabled);
        uint8_t getFullScaleAccelRange();
        void setFullScaleAccelRange(uint8_t range);
		
		// ACCEL_CONFIG2 register (TODO)
		
		// LP_MODE_CFG register
		//bool getGyroWakeCycleEnabled();
		//void setGyroWakeCycleEnabled(bool enabled);
		//uint8_t getGyroAvgCfg();
		//void setGyroAvgCfg(uint8_t averages);
		
		// ACCEL_WOM_THR register
		//uint8_t getWOMThreshold();
		//void setWOMThreshold(uint8_t threshold);

        // FIFO_EN register
        bool getTempFIFOEnabled();
        void setTempFIFOEnabled(bool enabled);
        bool getXGyroFIFOEnabled();
        void setXGyroFIFOEnabled(bool enabled);
        bool getYGyroFIFOEnabled();
        void setYGyroFIFOEnabled(bool enabled);
        bool getZGyroFIFOEnabled();
        void setZGyroFIFOEnabled(bool enabled);
        bool getAccelFIFOEnabled();
        void setAccelFIFOEnabled(bool enabled);
		
		// FSYNC_INT register
		//bool getFSYNCIntStatus();

        // INT_PIN_CFG register
        bool getInterruptMode();
        void setInterruptMode(bool mode);
        bool getInterruptDrive();
        void setInterruptDrive(bool drive);
        bool getInterruptLatch();
        void setInterruptLatch(bool latch);
        bool getInterruptLatchClear();
        void setInterruptLatchClear(bool clear);
        bool getFSyncInterruptLevel();
        void setFSyncInterruptLevel(bool level);
        bool getFSyncInterruptEnabled();
        void setFSyncInterruptEnabled(bool enabled);

        // INT_ENABLE register
		//bool getWOMInterruptEnabled();
		//void setWOMInterruptEnabled(bool enabled);
        //bool getFIFOOverflowInterruptEnabled();
		//void setFIFOOverflowInterruptEnabled(bool enabled);
		//bool getGDriveInterruptEnabled();
		//void setGDriveInterruptEnabled(bool enabled);
		//bool getDMPInterruptEnabled();
		//void setDMPInterruptEnabled(bool enabled);
		//bool getDataRdyInterruptEnabled();
		//void setDataRdyInterruptEnabled(bool enabled);
		
		// DMP_INT_STATUS register
		uint8_t clearDMPInterrupt();
		
		// INT_STATUS register
		//bool getWOMInterruptStatus();
        bool getFIFOOverflowInterruptStatus();
		//bool getGDriveInterruptStatus();
		//bool getDMPInterruptStatus();
		bool getDataReadyInterruptStatus();

        // ACCEL_*OUT_* registers
        void getMotion6(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz);
        void getAcceleration(int16_t* x, int16_t* y, int16_t* z);
        int16_t getAccelerationX();
        int16_t getAccelerationY();
        int16_t getAccelerationZ();

        // TEMP_OUT_* registers
        int16_t getTemperature();

        // GYRO_*OUT_* registers
        void getRotation(int16_t* x, int16_t* y, int16_t* z);
        int16_t getRotationX();
        int16_t getRotationY();
        int16_t getRotationZ();
		
		// SIGNAL_PATH_RESET register
        void resetAccelerometerPath();
        void resetTemperaturePath();
		
		// ACCEL_INTEL_CTRL register
		//bool getAccelIntelEnabled();
		//void setAccelIntelEnabled(bool enabled);
		//bool getAccelIntelMode();
		//void setAccelIntelMode(bool mode);

        // USER_CTRL register
        bool getFIFOEnabled();
        void setFIFOEnabled(bool enabled);
        void switchSPIEnabled(bool enabled);
        void resetFIFO();
        void resetSensors();

        // PWR_MGMT_1 register
        void reset();
        bool getSleepEnabled();
        void setSleepEnabled(bool enabled);
        bool getAccelWakeCycleEnabled();
        void setAccelWakeCycleEnabled(bool enabled);
		//bool getGyroStandbyEnabled();
		//void setGyroStandbyEnabled(bool enabled);
        bool getTempSensorEnabled();
        void setTempSensorEnabled(bool enabled);
        uint8_t getClockSource();
        void setClockSource(uint8_t source);

        // PWR_MGMT_2 register
        //bool getFIFOLowPowerModeEnabled();
		//void setFIFOLowPowerModeEnabled(bool enabled);
		//bool getDMPLowPowerModeEnabled();
		//void setDMPLowPowerModeEnabled(bool enabled);
        bool getStandbyXAccelEnabled();
        void setStandbyXAccelEnabled(bool enabled);
        bool getStandbyYAccelEnabled();
        void setStandbyYAccelEnabled(bool enabled);
        bool getStandbyZAccelEnabled();
        void setStandbyZAccelEnabled(bool enabled);
        bool getStandbyXGyroEnabled();
        void setStandbyXGyroEnabled(bool enabled);
        bool getStandbyYGyroEnabled();
        void setStandbyYGyroEnabled(bool enabled);
        bool getStandbyZGyroEnabled();
        void setStandbyZGyroEnabled(bool enabled);

        // FIFO_COUNT_* registers
        uint16_t getFIFOCount();

        // FIFO_R_W register
        uint8_t getFIFOByte();
        void setFIFOByte(uint8_t data);
        void getFIFOBytes(uint8_t *data, uint8_t length);

        // WHO_AM_I register
        uint8_t getDeviceID();
        void setDeviceID(uint8_t id);

    private:
        uint8_t devAddr;
        uint8_t buffer[14];
};

#endif /* _ICM20689_H_ */