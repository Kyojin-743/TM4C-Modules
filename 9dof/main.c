/**
 * main.c
 */


/* INCLUDES */

//C-Std Lib.
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <math.h>

//TM4C-Core
#include "tm4c123gh6pm.h"
#include "clock.h"
#include "wait.h"
#include "nvic.h"

//TM4C-Peripheral drivers
#include "gpio.h"
#include "i2c1.h"
#include "uart0.h"

//Higher Level Peripheral Access
#include "shell.h"
#include "mpu6050.h"
#include "qmc5883p.h"
#include "quaternion.h"

/* GLOBALS CONSTS AND MACROS */

#define LED_G           PORTF,3
#define MPU6050_INT     PORTE,4
#define QMC5883P_INT    PORTB,5

//float mag_A[3][3] = {{1.02194020, 0.01561863, -0.00040929},
//                     {0.01561863, 0.97946220, -0.01384287},
//                     {-0.00040929, -0.01384287, 0.99948832}};
//float mag_b[3] = {0.01092655, -0.00538835, -0.02026989};

float mag_A[3][3] = {{0.00861610, 0.00195565, 0.00083649},
                    {0.00195565, 10.99483483, -0.22468925},
                    {0.00083649, -0.22468925, 10.5611354}};

float mag_b [3]= {178.46403898, -0.03880254, -0.03374748};

float accel_A[3][3] = {{1.00550042, -0.00211106, -0.00102880},
                       {-0.00211106, 1.00552839, 0.00102497},
                       {-0.00102880, 0.00102497, 0.98906820}};
float accel_b[3] = {0.02526037, 0.00277766, 0.01647459};


/* SUB ROUTINE PROTOTYPES */

void initHw(void);
void printData(Vec3f a, Vec3f g, Vec3f m);
void printVec(Vec3f v);
float deltaSeconds(void);
uint32_t start(void);
uint32_t stop(void);
float seconds(uint32_t counts);
Vec3f complimentaryFilter(Vec3f a, Vec3f g, float dt_s);
Vec3f IMU_AHRS_update(Quaternion *q_est, const Vec3f *a, const Vec3f *g, float dt_s);
Vec3f MARG_AHRS_update(Quaternion *q_est, const Vec3f *a, const Vec3f *g, const Vec3f *m, float dt_s);
Vec3f gyroOffsets(void);
void filter_loop(Vec3f g_ofs);
void test_loop(Vec3f g_ofs);
void marg_loop(Vec3f g_ofs);
Vec3f sliding_window(Vec3f window[], Vec3f new_data, uint8_t size, uint8_t *window_idx);
Vec3f correct(Vec3f raws, float A[3][3], float b[3]);

#define WINDOW_SIZE 4

/* MAIN ROUTINE */
int main(void) {
    initHw();
    Vec3f g_ofs = gyroOffsets();
    for(;;) {

//        filter_loop(g_ofs);
//        test_loop(g_ofs);
        marg_loop(g_ofs);

//        waitMicrosecond(40e3);
    }
}


/* SUB ROUTINES */

void initHw(void) {
    initSystemClockTo40Mhz();
    initSystemClockTo80Mhz();

    //GPIO
    enablePort(PORTF);
    enablePort(PORTE);
    enablePort(PORTB);
    selectPinPushPullOutput(LED_G);
    selectPinDigitalInput(QMC5883P_INT);
    selectPinDigitalInput(MPU6050_INT);

    //Shell
    initUart0();
    setUart0BaudRate(115200, 80e6);

    //Sensor
    initI2c1();
    mpu_init();
    qmc_init();

    //Startup Sequence
    putsUart0(CLEAR_COLOR);
    putsUart0(CLEAR_SCREEN);
    putsUart0(GOTO_HOME);
    putsUart0(HIDE_CURSOR);
    setPinValue(LED_G, 1);
    waitMicrosecond(500e3*2);
    setPinValue(LED_G, 0);

    //Stopwatch
    SYSCTL_RCGCWTIMER_R |= SYSCTL_RCGCWTIMER_R0;
    _delay_cycles(3);

    WTIMER0_CTL_R  &= ~(TIMER_CTL_TAEN);
    WTIMER0_CFG_R   = 0x4;
    WTIMER0_TAMR_R  = TIMER_TAMR_TAMR_1_SHOT | TIMER_TAMR_TACDIR;
    WTIMER0_TAV_R   = 0;
    WTIMER0_CTL_R  |= TIMER_CTL_TAEN;

    WTIMER0_CTL_R  &= ~(TIMER_CTL_TBEN);
    WTIMER0_CFG_R   = 0x4;
    WTIMER0_TBMR_R  = TIMER_TBMR_TBMR_1_SHOT | TIMER_TBMR_TBCDIR;
    WTIMER0_TBV_R   = 0;
    WTIMER0_CTL_R  |= TIMER_CTL_TBEN;
}

Vec3f correct(Vec3f raw, float A[3][3], float b[3]) {
    return (Vec3f) {
        .x=A[0][0] * raw.x + A[0][1] * raw.y + A[0][2] * raw.z + b[0],
        .y=A[1][0] * raw.x + A[1][1] * raw.y + A[1][2] * raw.z + b[1],
        .z=A[2][0] * raw.x + A[2][1] * raw.y + A[2][2] * raw.z + b[2]
    };
}

Vec3f sliding_window(Vec3f window[], Vec3f new_data, uint8_t size, uint8_t *window_idx) {
    window[((*window_idx)++) & (size-1)] = new_data;
    uint8_t i;
    Vec3f v = {};
    for(i = 0; i < size; ++i) {
        v.x += window[i].x;
        v.y += window[i].y;
        v.z += window[i].z;
    }
    v.x /= (float)size;
    v.y /= (float)size;
    v.z /= (float)size;
    return v;
}

void test_loop(Vec3f g_ofs) {
    static Vec3f window_a[WINDOW_SIZE] = {};
    static Vec3f window_g[WINDOW_SIZE] = {};
    static Vec3f window_m[WINDOW_SIZE] = {};
    static uint8_t idx_a = 0, idx_g = 0, idx_m = 0;

    char buffer[100];

    MpuData mpu = mpu_read();
    Vec3f accel = (Vec3f){mpu.a.x, mpu.a.y, mpu.a.z};
    Vec3f gyro  = (Vec3f){mpu.g.x - g_ofs.x, mpu.g.y - g_ofs.y, mpu.g.z - g_ofs.z};
    Vec3f mag   = qmc_read();

    float t = mag.x;
    mag.x = mag.y;
    mag.y = -t;

    accel = correct(accel, accel_A, accel_b);

    accel = sliding_window(window_a, accel, WINDOW_SIZE, &idx_a);
    gyro = sliding_window(window_g, gyro, WINDOW_SIZE, &idx_g);
    mag = sliding_window(window_m, mag, WINDOW_SIZE, &idx_m);

//    putsUart0(SAVE_POS);
    usprintf(buffer, "%f,%f,%f\n", mag.x, mag.y, mag.z);
    putsUart0(buffer);
//    putsUart0(RETURN_2_POS);
}

void filter_loop(Vec3f g_ofs) {
    static Vec3f window_a[WINDOW_SIZE] = {};
    static Vec3f window_g[WINDOW_SIZE] = {};
    static Vec3f window_m[WINDOW_SIZE] = {};
    static Quaternion imu_est = {1.0f, 0.0f, 0.0f, 0.0f};
    static Quaternion marg_est = {1.0f, 0.0f, 0.0f, 0.0f};
    static uint8_t idx_a = 0, idx_g = 0, idx_m = 0;

    uint32_t begin, end;
    char buffer[100];

    putsUart0(SAVE_POS);
    MpuData mpu = mpu_read();
    Vec3f accel = (Vec3f){mpu.a.x, mpu.a.y, mpu.a.z};
    Vec3f gyro  = (Vec3f){mpu.g.x - g_ofs.x, mpu.g.y - g_ofs.y, mpu.g.z - g_ofs.z};
    Vec3f mag   = qmc_read();

    accel = sliding_window(window_a, accel, WINDOW_SIZE, &idx_a);
    gyro = sliding_window(window_g, gyro, WINDOW_SIZE, &idx_g);
    mag = sliding_window(window_m, mag, WINDOW_SIZE, &idx_m);

    float dt_s = deltaSeconds();
    Print("---------------------Raws-----------------", .bg=Gray);
    printData(accel, gyro, mag);
    usprintf(buffer, "dt: %f\n", dt_s);
    putsUart0(buffer);


    Print("-----------------Complimentary------------", .bg=Gray);
    begin = start();
    Vec3f c_att = complimentaryFilter(accel, gyro, dt_s);
    end = stop();
    printVec(c_att);
    usprintf(buffer, "Time: %fus | Clocks: %d\n", seconds(end-begin)*1e6, end-begin);
    putsUart0(buffer);


    Print("----------------Madgwick 6DoF-------------", .bg=Gray);
    begin = start();
    Vec3f m6_att = IMU_AHRS_update(&imu_est, &accel, &gyro, dt_s);
    end = stop();
    printVec(m6_att);
    usprintf(buffer, "Time: %fus | Clocks: %d\n", seconds(end-begin)*1e6, end-begin);
    putsUart0(buffer);


    Print("----------------Madgwick 9DoF-------------", .bg=Gray);
    begin = start();
    Vec3f m9_att = MARG_AHRS_update(&marg_est, &accel, &gyro, &mag, dt_s);
    end = stop();
    printVec(m9_att);
    usprintf(buffer, "Time: %fus | Clocks: %d\n", seconds(end-begin)*1e6, end-begin);
    putsUart0(buffer);

    putsUart0(RETURN_2_POS);
}

void marg_loop(Vec3f g_ofs) {
    static Vec3f window_a[WINDOW_SIZE] = {};
    static Vec3f window_g[WINDOW_SIZE] = {};
    static Vec3f window_m[WINDOW_SIZE] = {};
    static uint8_t idx_a = 0, idx_g = 0, idx_m = 0;
    static float dt_marg_s = 0.0f;
    static Quaternion q_est = {1.0f, 0.0f, 0.0f, 0.0f};

    //Only update when mpu data is valid
    if(!getPinValue(MPU6050_INT)) return;
    uint8_t data = readI2c1Register(QMC5883P_ADDR, QMC5883P_STAT_R);

    char buffer[100];

    MpuData mpu = mpu_read();
    Vec3f accel = (Vec3f){mpu.a.x, mpu.a.y, mpu.a.z};
    Vec3f gyro  = (Vec3f){mpu.g.x - g_ofs.x, mpu.g.y - g_ofs.y, -(mpu.g.z - g_ofs.z)};
    Vec3f mag;
    if(data & QMC5883P_STAT_DRDY) {
        mag = qmc_read();
        float t = mag.x;
        mag.x = mag.y;
        mag.y = -t;
        mag = correct(mag, mag_A, mag_b);
        mag = sliding_window(window_m, mag, WINDOW_SIZE, &idx_m);
    }
    float dt_s = deltaSeconds();
    dt_marg_s += dt_s;

    accel = correct(accel, accel_A, accel_b);
    accel   = sliding_window(window_a, accel, WINDOW_SIZE, &idx_a);
    gyro    = sliding_window(window_g, gyro, WINDOW_SIZE, &idx_g);

    Vec3f euler;
    //Run the Full 9DOF MARG AHRS Update
    if(data & QMC5883P_STAT_DRDY) {
        if(data & QMC5883P_STAT_OVFL) return;//if overflow -> skip
        setPinValue(LED_G, 1);
        euler = MARG_AHRS_update(&q_est, &accel, &gyro, &mag, dt_marg_s);
        dt_marg_s = 0;
    }
    //Run the Partial 6DOF IMU AHRS Update
    else {
        setPinValue(LED_G, 0);
        euler = IMU_AHRS_update(&q_est, &accel, &gyro, dt_s);
        //dt_s 'auto' zeros after function scope ends
    }

    //serial Debug
    usprintf(buffer, "%f,%f,%f\n", euler.x, euler.y, euler.z);
    putsUart0(buffer);
}

//#define COUNT2SEC .0000000250 //40MHz
#define COUNT2SEC .0000000125 //80MHz
float deltaSeconds(void) {
    uint32_t counts = WTIMER0_TAV_R;
    WTIMER0_TAV_R = 0;
    return ((float)counts) * COUNT2SEC;
}

uint32_t start(void) {
    return WTIMER0_TBV_R;
}

uint32_t stop(void) {
    uint32_t counts = WTIMER0_TBV_R;
    WTIMER0_TBV_R = 0;
    return counts;
}

float seconds(uint32_t counts) {
    return ((float)counts) * COUNT2SEC;
}

Vec3f gyroOffsets(void) {
    uint32_t i;
    MpuData m;
    Vec3f s = {0,0,0};
    for(i = 0; i < 1000; ++i) {
        m = mpu_read();
        s.x += m.g.x;
        s.y += m.g.y;
        s.z += m.g.z;
        waitMicrosecond(2e3);
    }
    s.x /= 1000;
    s.y /= 1000;
    s.z /= 1000;
    return s;
}

void printData(Vec3f a, Vec3f g, Vec3f m) {
    char buffer[100];
    usprintf(buffer, "Ax:%10f|Ay:%10f|Az:%10f|\n", a.x, a.y, a.z);
    putsUart0(buffer);
    usprintf(buffer, "Gx:%10f|Gy:%10f|Gz:%10f|\n", g.x, g.y, g.z);
    putsUart0(buffer);
    usprintf(buffer, "Mx:%10f|My:%10f|Mz:%10f|\n", m.x, m.y, m.z);
    putsUart0(buffer);
}

void printVec(Vec3f v) {
    char buffer[100];
    usprintf(buffer, "X:%10f|Y:%10f|Z:%10f   |\n", v.x, v.y, v.z);
//    putsUart0(CLEAR_LINE);
    putsUart0(buffer);
}

#define C_ALPHA   0.95
Vec3f complimentaryFilter(Vec3f a, Vec3f g, float dt_s) {
    static Vec3f gyro_est = {0, 0, 0};
    Vec3f accel_est = {0 ,0, 0};
    //integrate Gyro
    gyro_est.x += g.x * dt_s;
    gyro_est.y += g.y * dt_s;
    gyro_est.z += g.z * dt_s;
    //Euler Angle from Accel
    accel_est.x = atan2f(a.y, sqrtf(a.x*a.x + a.z*a.z));
    accel_est.y = atan2f(a.x, sqrtf(a.y*a.y + a.z*a.z));
    accel_est.z = atan2f(sqrtf(a.x*a.x + a.y*a.y), a.z);
    return (Vec3f){
        .x= (C_ALPHA)*gyro_est.x + (1-C_ALPHA)*accel_est.x,
        .y= (C_ALPHA)*gyro_est.y + (1-C_ALPHA)*accel_est.y,
        .z= (C_ALPHA)*gyro_est.z + (1-C_ALPHA)*accel_est.z
    };
}

#define M_BETA   .02f
#define RAD2DEG  57.2957795131f
#define DEG2RAD  0.0174532925f

//============================================================
//9-DOF MADGWICK
//============================================================
void MadgwickQuaternionUpdate(float q[4], float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float deltat);

Vec3f MARG_AHRS_update(Quaternion *q_est, const Vec3f *a, const Vec3f *g, const Vec3f *m, float dt_s) {
    float q[4];
    q[0] = q_est->w;
    q[1] = q_est->x;
    q[2] = q_est->y;
    q[3] = q_est->z;

    Quaternion q_prev = *q_est;

    float ax = a->x, ay = a->y, az = a->z,
          gx = g->x * DEG2RAD, gy = g->y * DEG2RAD, gz = g->z * DEG2RAD,
          mx = m->x, my = m->y, mz = m->z;
    MadgwickQuaternionUpdate(q, ax, ay, az, gx, gy, gz, mx, my, mz, dt_s);
    q_est->w = q[0];
    q_est->x = q[1];
    q_est->y = q[2];
    q_est->z = q[3];

    float dot = q_est->w * q_prev.w + q_est->x * q_prev.x + q_est->y * q_prev.y + q_est->z * q_prev.z;
    if (dot < 0.0f) {
        q_est->w = -q_est->w;
        q_est->x = -q_est->x;
        q_est->y = -q_est->y;
        q_est->z = -q_est->z;
    }

    float r11 = 1 - 2*(q_est->y*q_est->y + q_est->z*q_est->z);
    float r21 = 2*(q_est->x*q_est->y - q_est->w*q_est->z);
    float r31 = 2*(q_est->x*q_est->z + q_est->w*q_est->y);
    float r32 = 2*(q_est->y*q_est->z - q_est->w*q_est->x);
    float r33 = 1 - 2*(q_est->x*q_est->x + q_est->y*q_est->y);

    return (Vec3f) {
        RAD2DEG * atan2f(r32, r33),
        RAD2DEG * asinf(-r31),
        RAD2DEG * atan2f(r21, r11)
    };

//    // My SLOW version for understanding:

//    // Sensor data
//    Quaternion qa = {0, a.x, a.y, a.z};
//    Quaternion qw = {0, g.x, g.y, g.z};
//    Quaternion qm = {0, m.x, m.y, m.z};
//
//    // Normalise Accel and Mag Sensor Data, convert Gyro Data
//    qa = q_norm(qa);
//    qm = q_norm(qm);
//    qw = q_scale(qw, DEG2RAD);
//
//    // Rotate Mag Reading from body frame to world frame
//    Quaternion h = q_mul(qp, q_mul(qm, q_star(qp)));
//    Quaternion b = (Quaternion) {
//        .w=0.0f,
//        .x=sqrtf(h.x*h.x + h.y*h.y),
//        .y=0.0f,
//        .z=h.z
//    };
//
//    //Form Objective functions for g and b with respective jacobians
//    // J_g,b^T = [ J_g^T | J_b^T]
//    // F_g,b   = [F_g | F_b ]^T (transpose cus i have to write as row vector but F is actually column vector
//    float F_g[3] = {
//        2*(qp.x*qp.z - qp.w*qp.y) - qa.x,
//        2*(qp.w*qp.x + qp.y*qp.z) - qa.y,
//        2*(0.5 - qp.x*qp.x - qp.y*qp.y) - qa.z
//    };
//
//    float J_g[3][4] = {
//       {-2*qp.y, 2*qp.z, -2*qp.w, 2*qp.x},
//       {2*qp.x, 2*qp.w, 2*qp.z, 2*qp.y},
//       {0, -4*qp.x, -4*qp.y, 0},
//    };
//
//    float F_b[3] = {
//        2*b.x*(0.5 - qp.y*qp.y  - qp.z*qp.z) + 2*b.z*(      qp.x*qp.z - qp.w*qp.y) - qm.x,
//        2*b.x*(      qp.x*qp.y  - qp.w*qp.z) + 2*b.z*(      qp.w*qp.x + qp.y*qp.z) - qm.y,
//        2*b.x*(      qp.w*qp.y  + qp.x*qp.z) + 2*b.z*(0.5 - qp.x*qp.x - qp.y*qp.y) - qm.z
//    };
//
//    float J_b[3][4] = {
//       {-2*b.z*qp.y, 2*b.z*qp.z, -4*b.x*qp.y - 2*b.z*qp.w, -4*b.x*qp.z + 2*b.z*qp.x},
//       {-2*b.x*qp.z + 2*b.z*qp.x, 2*b.x*qp.y + 2*b.z*qp.w, 2*b.x*qp.x + 2*b.z*qp.z, -2*b.x*qp.w + 2*b.z*qp.y},
//       {2*b.x*qp.y, 2*b.x*qp.z - 4*b.z*qp.x, 2*b.x*qp.w - 4*b.z*qp.y, 2*b.x*qp.x},
//    };
//
//    // qga = J_g^T * F_g
//    Quaternion qga;
//    qga.w = J_g[0][0]*F_g[0] + J_g[1][0]*F_g[1] + J_g[2][0]*F_g[2];
//    qga.x = J_g[0][1]*F_g[0] + J_g[1][1]*F_g[1] + J_g[2][1]*F_g[2];
//    qga.y = J_g[0][2]*F_g[0] + J_g[1][2]*F_g[1] + J_g[2][2]*F_g[2];
//    qga.z = J_g[0][3]*F_g[0] + J_g[1][3]*F_g[1] + J_g[2][3]*F_g[2];
//
//    // qgb = J_b^T * F_b
//    Quaternion qgb;
//    qgb.w = J_b[0][0]*F_b[0] + J_b[1][0]*F_b[1] + J_b[2][0]*F_b[2];
//    qgb.x = J_b[0][1]*F_b[0] + J_b[1][1]*F_b[1] + J_b[2][1]*F_b[2];
//    qgb.y = J_b[0][2]*F_b[0] + J_b[1][2]*F_b[1] + J_b[2][2]*F_b[2];
//    qgb.z = J_b[0][3]*F_b[0] + J_b[1][3]*F_b[1] + J_b[2][3]*F_b[2];
//
//    //Gradiante (J_g,b^T * F_g,b) equates to J_g^T * F_g + J_b^T * F_b
//    Quaternion qg = q_add(qga, qgb);
//
//    // normalise step magnitude
//    qg = q_scale(q_norm(qg), M_BETA);
//
//    //compute q_dot
//    Quaternion qd = q_sub(q_mul(q_scale(qp, 0.5), qw), qg);
//
//    //Integrate and normalize to get q_est
//    q_est = q_norm(q_add(q_est, q_scale(qd, dt_s)));
//
//    // Form Rotation Matrix
//    float r11 = 1 - 2*(q_est.y*q_est.y + q_est.z*q_est.z);
//    float r21 = 2*(q_est.x*q_est.y - q_est.w*q_est.z);
//    float r31 = 2*(q_est.x*q_est.z + q_est.w*q_est.y);
//    float r32 = 2*(q_est.y*q_est.z - q_est.w*q_est.x);
//    float r33 = 1 - 2*(q_est.x*q_est.x + q_est.y*q_est.y);
//
//    // Return as Euler Angles
//    return (Vec3f) {
//        RAD2DEG * atan2f(r32, r33),
//        RAD2DEG * asinf(-r31),
//        RAD2DEG * atan2f(r21, r11)
//    };
}
#undef M_BETA
#undef RAD2DEG
#undef DEG2RAD

//============================================================
//6-DOF MADGWICK
//============================================================

#define M_BETA       0.01f
#define RAD2DEG     57.2957795131
#define DEG2RAD      0.0174532925
Vec3f IMU_AHRS_update(Quaternion *q_est, const Vec3f *a, const Vec3f *g, float dt_s) {
    Quaternion q_prev = *q_est;

    //Estimate
    Quaternion q_accel = {0, a->x, a->y, a->z};
    q_accel = q_norm(q_accel);
    Quaternion q_gyro = {0, g->x*.5, g->y*.5, g->z*.5};
    q_gyro = q_scale(q_gyro, DEG2RAD);
    q_gyro = q_mul(q_prev, q_gyro);

    //Objective Function
    float f[3] = {
          2*(q_prev.x*q_prev.z - q_prev.w*q_prev.y) - q_accel.x,
          2*(q_prev.w*q_prev.x + q_prev.y*q_prev.z) - q_accel.y,
          2*(0.5 - q_prev.x*q_prev.x - q_prev.y*q_prev.y) - q_accel.z,
    };

    float j[3][4] =  {
          {-2 * q_prev.y, 2 * q_prev.z, -2 * q_prev.w, 2 * q_prev.x},
          {2 * q_prev.x, 2 * q_prev.w, 2 * q_prev.z, 2 * q_prev.y},
          {0, -4 * q_prev.x, -4 * q_prev.y, 0},
    };

    //Gradiant (J' * F)
    Quaternion q_grad = {
         j[0][0]*f[0] + j[1][0]*f[1] + j[2][0]*f[2],
         j[0][1]*f[0] + j[1][1]*f[1] + j[2][1]*f[2],
         j[0][2]*f[0] + j[1][2]*f[1] + j[2][2]*f[2],
         j[0][3]*f[0] + j[1][3]*f[1] + j[2][3]*f[2]
    };
    q_grad = q_norm(q_grad);
    q_grad = q_scale(q_grad, M_BETA);
    *q_est = q_add(q_prev, q_scale(q_sub(q_gyro, q_grad), dt_s));

    float dot = q_est->w * q_prev.w + q_est->x * q_prev.x + q_est->y * q_prev.y + q_est->z * q_prev.z;
    if (dot < 0.0f) {
        q_est->w = -q_est->w;
        q_est->x = -q_est->x;
        q_est->y = -q_est->y;
        q_est->z = -q_est->z;
    }

    // Rotation matrix (body to world, assuming ZYX order)
    float r11 = 1 - 2*(q_est->y*q_est->y + q_est->z*q_est->z);
    float r21 = 2*(q_est->x*q_est->y - q_est->w*q_est->z);
    float r31 = 2*(q_est->x*q_est->z + q_est->w*q_est->y);
    float r32 = 2*(q_est->y*q_est->z - q_est->w*q_est->x);
    float r33 = 1 - 2*(q_est->x*q_est->x + q_est->y*q_est->y);

    return (Vec3f) {
        RAD2DEG * atan2f(r32, r33),
        RAD2DEG * asinf(-r31),
        RAD2DEG * atan2f(r21, r11)
    };
}
#undef M_BETA
#undef RAD2DEG
#undef DEG2RAD

//============================================================================
//============================================================================
// Using for speed
//============================================================================
//============================================================================

#define M_BETA  0.2f
void MadgwickQuaternionUpdate(float q[4], float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float deltat)
{
    float beta = M_BETA;
    float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];   // short name local variable for readability
    float norm;
    float hx, hy, _2bx, _2bz;
    float s1, s2, s3, s4;
    float qDot1, qDot2, qDot3, qDot4;

    // Auxiliary variables to avoid repeated arithmetic
    float _2q1mx;
    float _2q1my;
    float _2q1mz;
    float _2q2mx;
    float _4bx;
    float _4bz;
    float _2q1 = 2.0f * q1;
    float _2q2 = 2.0f * q2;
    float _2q3 = 2.0f * q3;
    float _2q4 = 2.0f * q4;
    float _2q1q3 = 2.0f * q1 * q3;
    float _2q3q4 = 2.0f * q3 * q4;
    float q1q1 = q1 * q1;
    float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q1q4 = q1 * q4;
    float q2q2 = q2 * q2;
    float q2q3 = q2 * q3;
    float q2q4 = q2 * q4;
    float q3q3 = q3 * q3;
    float q3q4 = q3 * q4;
    float q4q4 = q4 * q4;

    // Normalise accelerometer measurement
    norm = 1.0f / sqrtf(ax * ax + ay * ay + az * az);
    ax *= norm;
    ay *= norm;
    az *= norm;

    // Normalise magnetometer measurement
    norm = 1.0f / sqrtf(mx * mx + my * my + mz * mz);
    mx *= norm;
    my *= norm;
    mz *= norm;

    // Reference direction of Earth's magnetic field
    _2q1mx = 2.0f * q1 * mx;
    _2q1my = 2.0f * q1 * my;
    _2q1mz = 2.0f * q1 * mz;
    _2q2mx = 2.0f * q2 * mx;
    hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4;
    hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4;
    _2bx = sqrtf(hx * hx + hy * hy);
    _2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4;
    _4bx = 2.0f * _2bx;
    _4bz = 2.0f * _2bz;

    // Gradient decent algorithm corrective step
    s1 = -_2q3 * (2.0f * q2q4 - _2q1q3 - ax) + _2q2 * (2.0f * q1q2 + _2q3q4 - ay) - _2bz * q3 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
    s2 = _2q4 * (2.0f * q2q4 - _2q1q3 - ax) + _2q1 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q2 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + _2bz * q4 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
    s3 = -_2q1 * (2.0f * q2q4 - _2q1q3 - ax) + _2q4 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q3 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
    s4 = _2q2 * (2.0f * q2q4 - _2q1q3 - ax) + _2q3 * (2.0f * q1q2 + _2q3q4 - ay) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
    norm = sqrtf(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);    // normalise step magnitude
    norm = 1.0f/norm;
    s1 *= norm;
    s2 *= norm;
    s3 *= norm;
    s4 *= norm;

    // Compute rate of change of quaternion
    qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz) - beta * s1;
    qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy) - beta * s2;
    qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx) - beta * s3;
    qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx) - beta * s4;

    // Integrate to yield quaternion
    q1 += qDot1 * deltat;
    q2 += qDot2 * deltat;
    q3 += qDot3 * deltat;
    q4 += qDot4 * deltat;
    norm = sqrtf(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
    norm = 1.0f/norm;
    q[0] = q1 * norm;
    q[1] = q2 * norm;
    q[2] = q3 * norm;
    q[3] = q4 * norm;

}
