//		Register name				Address	   I/F 		Bit7 Bit6 Bit5 Bit4 Bit3 Bit2 Bit1 Bit0
#define MPU_6050_SELF_TEST_X 		0x0D    // R/W 		XA_TEST[4-2] XG_TEST[4-0]
#define MPU_6050_SELF_TEST_Y 		0x0E 	// R/W 		YA_TEST[4-2] YG_TEST[4-0]
#define MPU_6050_SELF_TEST_Z 		0x0F 	// R/W 		ZA_TEST[4-2] ZG_TEST[4-0]
#define MPU_6050_SELF_TEST_A 		0x10 	// R/W 		RESERVED XA_TEST[1-0] YA_TEST[1-0] ZA_TEST[1-0]
#define MPU_6050_SMPLRT_DIV 		0x19 	// R/W 		SMPLRT_DIV[7:0]
#define MPU_6050_CONFIG 			0x1A 	// R/W 		EXT_SYNC_SET[2:0] DLPF_CFG[2:0]
#define MPU_6050_GYRO_CONFIG 		0x1B 	// R/W 		FS_SEL [1:0]
#define MPU_6050_ACCEL_CONFIG 		0x1C 	// R/W 		XA_ST YA_ST ZA_ST AFS_SEL[1:0]
#define MPU_6050_FIFO_EN 			0x23 	// R/W 		TEMP_FIFO_EN XG_FIFO_EN YG_FIFO_EN ZG_FIFO_EN ACCEL_FIFO_EN SLV2_FIFO_EN SLV1_FIFO_EN SLV0_FIFO_EN
#define MPU_6050_I2C_MST_CTRL 		0x24 	// R/W 		MULT_MST_EN WAIT_FOR_ES SLV_3_FIFO_EN I2C_MST_P_NSR 		I2C_MST_CLK[3:0]
#define MPU_6050_I2C_SLV0_ADDR 		0x25 	// R/W 		I2C_SLV0_RW I2C_SLV0_ADDR[6:0]
#define MPU_6050_I2C_SLV0_REG 		0x26 	// R/W 		I2C_SLV0_REG[7:0]
#define MPU_6050_I2C_SLV0_CTRL 		0x27 	// R/W 		I2C_SLV0_EN I2C_SLV0_BYTE_SW I2C_SLV0_REG_DIS I2C_SLV0_GRP I2C_SLV0_LEN[3:0]
#define MPU_6050_I2C_SLV1_ADDR 		0x28 	// R/W 		I2C_SLV1_RW I2C_SLV1_ADDR[6:0]
#define MPU_6050_I2C_SLV1_REG 		0x29 	// R/W 		I2C_SLV1_REG[7:0]
#define MPU_6050_I2C_SLV1_CTRL 		0x2A 	// R/W 		I2C_SLV1_EN I2C_SLV1_BYTE_SW I2C_SLV1_REG_DIS I2C_SLV1_GRP I2C_SLV1_LEN[3:0]
#define MPU_6050_I2C_SLV2_ADDR 		0x2B 	// R/W 		I2C_SLV2_RW I2C_SLV2_ADDR[6:0]
#define MPU_6050_I2C_SLV2_REG 		0x2C 	// R/W 		I2C_SLV2_REG[7:0]
#define MPU_6050_I2C_SLV2_CTRL 		0x2D 	// R/W 		I2C_SLV2_EN I2C_SLV2_BYTE_SW I2C_SLV2_REG_DIS I2C_SLV2_GRP I2C_SLV2_LEN[3:0]
#define MPU_6050_I2C_SLV3_ADDR 		0x2E 	// R/W 		I2C_SLV3_RW I2C_SLV3_ADDR[6:0]
#define MPU_6050_I2C_SLV3_REG 		0x2F 	// R/W 		I2C_SLV3_REG[7:0]
#define MPU_6050_I2C_SLV3_CTRL 		0x30 	// R/W 		I2C_SLV3_EN I2C_SLV3_BYTE_SW I2C_SLV3_REG_DIS I2C_SLV3_GRP I2C_SLV3_LEN[3:0]
#define MPU_6050_I2C_SLV4_ADDR 		0x31 	// R/W 		I2C_SLV4_RW I2C_SLV4_ADDR[6:0]
#define MPU_6050_I2C_SLV4_REG 		0x32 	// R/W 		I2C_SLV4_REG[7:0]
#define MPU_6050_I2C_SLV4_DO 		0x33 	// R/W 		I2C_SLV4_DO[7:0]
#define MPU_6050_I2C_SLV4_CTRL 		0x34 	// R/W 		I2C_SLV4_EN I2C_SLV4_INT_EN I2C_SLV4_REG_DIS I2C_MST_DLY[4:0]
#define MPU_6050_I2C_SLV4_DI 		0x35 	// R 		I2C_SLV4_DI[7:0]
#define MPU_6050_I2C_MST_STATUS 	0x36 	// R 		PASS_THROUGH I2C_SLV4_DONE I2C_LOST_ARB I2C_SLV4_NACK I2C_SLV3_NACK I2C_SLV2_NACK I2C_SLV1_NACK I2C_SLV0_NACK
#define MPU_6050_INT_PIN_CFG 		0x37 	// R/W 		INT_LEVEL INT_OPEN LATCH_INT_EN INT_RD_CLEAR 		FSYNC_INT_LEVEL FSYNC_INT_EN I2C_BYPASS_EN
#define MPU_6050_INT_ENABLE 		0x38 	// R/W 		FIFO_OFLOW_EN I2C_MST_INT_EN DATA_RDY_EN
#define MPU_6050_INT_STATUS 		0x3A	// R 		FIFO_OFLOW_INT I2C_MST_INT DATA_RDY_INT
#define MPU_6050_ACCEL_XOUT_H 		0x3B	// R 		ACCEL_XOUT[15:8]
#define MPU_6050_ACCEL_XOUT_L 		0x3C	// R 		ACCEL_XOUT[7:0]
#define MPU_6050_ACCEL_YOUT_H 		0x3D	// R 		ACCEL_YOUT[15:8]
#define MPU_6050_ACCEL_YOUT_L 		0x3E	// R 		ACCEL_YOUT[7:0]
#define MPU_6050_ACCEL_ZOUT_H 		0x3F	// R 		ACCEL_ZOUT[15:8]
#define MPU_6050_ACCEL_ZOUT_L 		0x40	// R 		ACCEL_ZOUT[7:0]
#define MPU_6050_TEMP_OUT_H 		0x41	// R 		TEMP_OUT[15:8]
#define MPU_6050_TEMP_OUT_L 		0x42	// R 		TEMP_OUT[7:0]
#define MPU_6050_GYRO_XOUT_H 		0x43	// R 		GYRO_XOUT[15:8]
#define MPU_6050_GYRO_XOUT_L 		0x44	// R 		GYRO_XOUT[7:0]
#define MPU_6050_GYRO_YOUT_H 		0x45	// R 		GYRO_YOUT[15:8]
#define MPU_6050_GYRO_YOUT_L 		0x46	// R 		GYRO_YOUT[7:0]
#define MPU_6050_GYRO_ZOUT_H 		0x47	// R 		GYRO_ZOUT[15:8]
#define MPU_6050_GYRO_ZOUT_L 		0x48	// R 		GYRO_ZOUT[7:0]
#define MPU_6050_EXT_SENS_DATA_00 	0x49	// R 		EXT_SENS_DATA_00[7:0]
#define MPU_6050_EXT_SENS_DATA_01 	0x4A	// R 		EXT_SENS_DATA_01[7:0]
#define MPU_6050_EXT_SENS_DATA_02 	0x4B	// R 		EXT_SENS_DATA_02[7:0]
#define MPU_6050_EXT_SENS_DATA_03 	0x4C	// R 		EXT_SENS_DATA_03[7:0]
#define MPU_6050_EXT_SENS_DATA_04 	0x4D	// R 		EXT_SENS_DATA_04[7:0]
#define MPU_6050_EXT_SENS_DATA_05 	0x4E	// R 		EXT_SENS_DATA_05[7:0]
#define MPU_6050_EXT_SENS_DATA_06 	0x4F	// R 		EXT_SENS_DATA_06[7:0]
#define MPU_6050_EXT_SENS_DATA_07 	0x50	// R 		EXT_SENS_DATA_07[7:0]
#define MPU_6050_EXT_SENS_DATA_08 	0x51	// R 		EXT_SENS_DATA_08[7:0]
#define MPU_6050_EXT_SENS_DATA_09 	0x52	// R 		EXT_SENS_DATA_09[7:0]
#define MPU_6050_EXT_SENS_DATA_10 	0x53	// R 		EXT_SENS_DATA_10[7:0]
#define MPU_6050_EXT_SENS_DATA_11 	0x54	// R 		EXT_SENS_DATA_11[7:0]
#define MPU_6050_EXT_SENS_DATA_12 	0x55	// R 		EXT_SENS_DATA_12[7:0]
#define MPU_6050_EXT_SENS_DATA_13 	0x56	// R 		EXT_SENS_DATA_13[7:0]
#define MPU_6050_EXT_SENS_DATA_14 	0x57	// R 		EXT_SENS_DATA_14[7:0]
#define MPU_6050_EXT_SENS_DATA_15 	0x58	// R 		EXT_SENS_DATA_15[7:0]
#define MPU_6050_EXT_SENS_DATA_16 	0x59	// R 		EXT_SENS_DATA_16[7:0]
#define MPU_6050_EXT_SENS_DATA_17 	0x5A	// R 		EXT_SENS_DATA_17[7:0]
#define MPU_6050_EXT_SENS_DATA_18 	0x5B	// R 		EXT_SENS_DATA_18[7:0]
#define MPU_6050_EXT_SENS_DATA_19 	0x5C	// R 		EXT_SENS_DATA_19[7:0]
#define MPU_6050_EXT_SENS_DATA_20 	0x5D	// R 		EXT_SENS_DATA_20[7:0]
#define MPU_6050_EXT_SENS_DATA_21 	0x5E	// R 		EXT_SENS_DATA_21[7:0]
#define MPU_6050_EXT_SENS_DATA_22 	0x5F	// R 		EXT_SENS_DATA_22[7:0]
#define MPU_6050_EXT_SENS_DATA_23 	0x60	// R 		EXT_SENS_DATA_23[7:0]
#define MPU_6050_I2C_SLV0_DO 		0x63	// R/W 		I2C_SLV0_DO[7:0]
#define MPU_6050_I2C_SLV1_DO 		0x64	// R/W 		I2C_SLV1_DO[7:0]
#define MPU_6050_I2C_SLV2_DO 		0x65	// R/W 		I2C_SLV2_DO[7:0]
#define MPU_6050_I2C_SLV3_DO 		0x66	// R/W 		I2C_SLV3_DO[7:0]
#define MPU_6050_I2C_MST_DELAY_CT 	0x67	// RL R/W 	DELAY_ES_SHADOW I2C_SLV4_DLY_EN I2C_SLV3_DLY_EN I2C_SLV2_DLY_EN I2C_SLV1_DLY_EN I2C_SLV0_DLY_EN
#define MPU_6050_SIGNAL_PATH_RES 	0x68	// ET R/W 	GYRO_RESET ACCEL_RESET TEMP_RESET
#define MPU_6050_USER_CTRL 			0x6A	// R/W 		FIFO_EN I2C_MST_EN I2C_IF_DIS FIFO_RESET I2C_MST_RESET SIG_COND_RESET
#define MPU_6050_PWR_MGMT_1 		0x6B	// R/W 		DEVICE_RESET SLEEP CYCLE TEMP_DIS CLKSEL[2:0]
#define MPU_6050_PWR_MGMT_2 		0x6C	// R/W 		LP_WAKE_CTRL[1:0] STBY_XA STBY_YA STBY_ZA STBY_XG STBY_YG STBY_ZG
#define MPU_6050_FIFO_COUNTH 		0x72	// R/W 		FIFO_COUNT[15:8]
#define MPU_6050_FIFO_COUNTL 		0x73	// R/W 		FIFO_COUNT[7:0]
#define MPU_6050_FIFO_R_W 			0x74	// R/W 		FIFO_DATA[7:0]
#define MPU_6050_WHO_AM_I 			0x75	// R 		WHO_AM_I[6:1]
