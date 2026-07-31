/*
 * mpu6050.h
 *
 *  Created on: Feb 13, 2026
 *      Author: kyojin
 */

#ifndef MPU6050_H_
#define MPU6050_H_


/* INCLUDES */

//C-Std Lib.
#include <stdint.h>
#include <stdbool.h>

/* GLOBALS CONSTS AND MACROS */

//MPU6050 Default I2C Address
#define MPU6050_ADDR        0x68

//MPU6050 Register Map

#define MPU6050_SMPLRT_DIV_R          0x19
#define MPU6050_CONFIG_R              0x1A
#define MPU6050_GYRO_CONFIG_R         0x1B
#define MPU6050_ACCEL_CONFIG_R        0x1C

#define MPU6050_MOT_THR_R             0x1F

#define MPU6050_FIFO_EN_R             0x23

#define MPU6050_I2C_MST_CTRL_R        0x24

#define MPU6050_I2C_SLV0_ADDR_R       0x25
#define MPU6050_I2C_SLV0_REG_R        0x26
#define MPU6050_I2C_SLV0_CTRL_R       0x27

#define MPU6050_I2C_SLV1_ADDR_R       0x28
#define MPU6050_I2C_SLV1_REG_R        0x29
#define MPU6050_I2C_SLV1_CTRL_R       0x2A

#define MPU6050_I2C_SLV2_ADDR_R       0x2B
#define MPU6050_I2C_SLV2_REG_R        0x2C
#define MPU6050_I2C_SLV2_CTRL_R       0x2D

#define MPU6050_I2C_SLV3_ADDR_R       0x2E
#define MPU6050_I2C_SLV3_REG_R        0x2F
#define MPU6050_I2C_SLV3_CTRL_R       0x30

#define MPU6050_I2C_SLV4_ADDR_R       0x31
#define MPU6050_I2C_SLV4_REG_R        0x32
#define MPU6050_I2C_SLV4_CTRL_R       0x34

#define MPU6050_I2C_MST_STATUS_R      0x36

#define MPU6050_INT_PIN_CFG_R         0x37
#define MPU6050_INT_ENABLE_R          0x38
#define MPU6050_DMP_INT_STATUS_R      0x39
#define MPU6050_INT_STATUS_R          0x3A

#define MPU6050_ACCEL_XOUT_H_R        0x3B
#define MPU6050_ACCEL_XOUT_L_R        0x3C
#define MPU6050_ACCEL_YOUT_H_R        0x3D
#define MPU6050_ACCEL_YOUT_L_R        0x3E
#define MPU6050_ACCEL_ZOUT_H_R        0x3F
#define MPU6050_ACCEL_ZOUT_L_R        0x40

#define MPU6050_TEMP_OUT_H_R          0x41
#define MPU6050_TEMP_OUT_L_R          0x42

#define MPU6050_GYRO_XOUT_H_R         0x43
#define MPU6050_GYRO_XOUT_L_R         0x44
#define MPU6050_GYRO_YOUT_H_R         0x45
#define MPU6050_GYRO_YOUT_L_R         0x46
#define MPU6050_GYRO_ZOUT_H_R         0x47
#define MPU6050_GYRO_ZOUT_L_R         0x48

#define MPU6050_I2C_SLV0_DO_R         0x63
#define MPU6050_I2C_SLV1_DO_R         0x64
#define MPU6050_I2C_SLV2_DO_R         0x65
#define MPU6050_I2C_SLV3_DO_R         0x66
#define MPU6050_I2C_MST_DELAY_CTRL_R  0x67

#define MPU6050_SIGNAL_PATH_RESET_R   0x68
#define MPU6050_MOT_DETECT_CTRL_R     0x69
#define MPU6050_USER_CTRL_R           0x6A

#define MPU6050_PWR_MGMT_1_R          0x6B
#define MPU6050_PWR_MGMT_2_R          0x6C
#define MPU6050_BANK_SEL_R            0x6D
#define MPU6050_MEM_START_ADDR_R      0x6E
#define MPU6050_MEM_R_W_R             0x6F

#define MPU6050_DMP_CFG_1_R           0x70
#define MPU6050_DMP_CFG_2_R           0x71
#define MPU6050_FIFO_COUNTH_R         0x72
#define MPU6050_FIFO_COUNTL_R         0x73
#define MPU6050_FIFO_R_W_R            0x74

#define MPU6050_WHO_AM_I_R            0x75

//Controls the Digital Low Pass Filter Cutoff frequency in Hz
#define MPU6050_CONFIG_DLPF_A260_G256       0x00
#define MPU6050_CONFIG_DLPF_A184_G188       0x01
#define MPU6050_CONFIG_DLPF_A94_G98         0x02
#define MPU6050_CONFIG_DLPF_A44_G42         0x03
#define MPU6050_CONFIG_DLPF_A21_G20         0x04
#define MPU6050_CONFIG_DLPF_A10_G10         0x05
#define MPU6050_CONFIG_DLPF_A5_G5           0x06

//Controls which external clock will drive sample rate
//  (recommended to be used)
#define MPU6050_CONFIG_FSYNC_DISABLE        0x00
#define MPU6050_CONFIG_FSYNC_TEMP           0x08
#define MPU6050_CONFIG_FSYNC_GYRO_X         0x10
#define MPU6050_CONFIG_FSYNC_GYRO_Y         0x18
#define MPU6050_CONFIG_FSYNC_GYRO_Z         0x20
#define MPU6050_CONFIG_FSYNC_ACCEL_X        0x28
#define MPU6050_CONFIG_FSYNC_ACCEL_Y        0x30
#define MPU6050_CONFIG_FSYNC_ACCEL_Z        0x38

//Setting this bit performs self test on cooresponding gyro axis
#define MPU6050_GYRO_CONFIG_Z_ST            0x02
#define MPU6050_GYRO_CONFIG_Y_ST            0x04
#define MPU6050_GYRO_CONFIG_X_ST            0x08

//Sets the Full scale Range in °/s
#define MPU6050_GYRO_CONFIG_FSSEL_250       0x00
#define MPU6050_GYRO_CONFIG_FSSEL_500       0x08
#define MPU6050_GYRO_CONFIG_FSSEL_1000      0x10
#define MPU6050_GYRO_CONFIG_FSSEL_2000      0x18

//Setting this bit performs self test on cooresponding accelerometer axis
#define MPU6050_ACCEL_CONFIG_Z_ST            0x02
#define MPU6050_ACCEL_CONFIG_Y_ST            0x04
#define MPU6050_ACCEL_CONFIG_X_ST            0x08

//Sets the Full scale Range in g's (gravity)
#define MPU6050_ACCEL_CONFIG_FSSEL_2        0x00
#define MPU6050_ACCEL_CONFIG_FSSEL_4        0x08
#define MPU6050_ACCEL_CONFIG_FSSEL_8        0x10
#define MPU6050_ACCEL_CONFIG_FSSEL_16       0x18

//Enables the FIFO for cooresponding value
//(Accel xyz grouped)
#define MPU6050_FIFO_EN_SLV0                0x01
#define MPU6050_FIFO_EN_SLV1                0x02
#define MPU6050_FIFO_EN_SLV2                0x04
#define MPU6050_FIFO_EN_ACCEL               0x08
#define MPU6050_FIFO_EN_GYRO_Z              0x10
#define MPU6050_FIFO_EN_GYRO_Y              0x20
#define MPU6050_FIFO_EN_GYRO_X              0x40
#define MPU6050_FIFO_EN_TEMP                0x80

//Configures the interrupt pin
#define MPU6050_INT_PIN_CFG_BYPASS_EN       0x02    //0: disabled    | 1: enabled
#define MPU6050_INT_PIN_CFG_FSYNC_EN        0x04    //0: disabled    | 1: enabled
#define MPU6050_INT_PIN_CFG_FSYNC_LVL       0x08    //0: active high | 1: active low
#define MPU6050_INT_PIN_CFG_RD_CLEAR        0x10    //0: clear by read of INT_STATUS
                                                    //1: clear by read of any register
#define MPU6050_INT_PIN_CFG_LATCH_EN        0x20    //0: Interrupt auto clears after 50us
                                                    //1: Interrupt is latched until read
#define MPU6050_INT_PIN_CFG_OD              0x40    //0: push/pull
                                                    //1: open-drain
#define MPU6050_INT_PIN_CFG_INT_LVL         0x80    //0: logic lvl HIGH
                                                    //1: logic lvl LOW

#define MPU6050_INT_ENABLE_DATA_READY       0x01
#define MPU6050_INT_ENABLE_I2C_MST          0x08
#define MPU6050_INT_ENABLE_OVRFLW           0x10
#define MPU6050_INT_ENABLE_MOTION           0x40

#define MPU6050_INT_STATUS_DATA_READY       0x01
#define MPU6050_INT_STATUS_I2C_MST          0x08
#define MPU6050_INT_STATUS_OVRFLW           0x10
#define MPU6050_INT_STATUS_MOTION           0x40

#define MPU6050_PWR_MGMT_1_CLKSEL_8M        0x0
#define MPU6050_PWR_MGMT_1_CLKSEL_GYR_X     0x01
#define MPU6050_PWR_MGMT_1_CLKSEL_GYR_Y     0x02
#define MPU6050_PWR_MGMT_1_CLKSEL_GYR_Z     0x03
#define MPU6050_PWR_MGMT_1_CLKSEL_32K_EXT   0x04
#define MPU6050_PWR_MGMT_1_CLKSEL_19M_EXT   0x05
#define MPU6050_PWR_MGMT_1_CLKSEL_STOP      0x07

#define MPU6050_PWR_MGMT_1_TEMP_DISABLE     0x08

#define MPU6050_PWR_MGMT_1_CYCLE            0x20
#define MPU6050_PWR_MGMT_1_SLEEP            0x40
#define MPU6050_PWR_MGMT_1_RESET            0x80

/* Structs & Enums & Types*/

typedef void (*func)(void);

#ifndef VEC3F
typedef struct { float x, y, z; } Vec3f;
#define VEC3F
#endif
typedef struct { Vec3f a, g; } MpuData;

#ifndef nullptr
#define nullptr ((void*)0)
#endif
/* SUB ROUTINE PROTOTYPES */

bool mpu_init(void);
MpuData mpu_read(void);

#endif /* MPU6050_H_ */
