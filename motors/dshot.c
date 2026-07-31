
#include <stdbool.h>
#include <gpio.h>
#include <nvic.h>

#define DSHOT_150
#include "dshot.h"
#include "tm4c123gh6pm.h"


#define DSHOT0A  PORTC,4
#define DSHOT0B  PORTC,5
#define DSHOT1A  PORTD,0
#define DSHOT1B  PORTD,1

#define min(a,b) ((a)>(b)?(b):(a))
#define max(a,b) ((a)<(b)?(b):(a))
#define clamp(x, a, b) min((b), max((a), (x)))
#define CRC(val) (((val)^((val)>>4)^((val)>>8))&0x0F)
static uint16_t createFrame(uint16_t throttle, bool telemetry) {
    uint16_t packet;
    uint8_t crc;

    throttle = clamp(throttle, 48, 2047);

    packet = (uint16_t)((throttle << 1) | (telemetry ? 1 : 0));
    crc = CRC(packet);
    packet = (uint16_t)((packet << 4) | crc);

    return packet;
}
#undef min
#undef max
#undef clamp
#undef CRC

void initDshot(void) {
    enablePort(PORTC);
    enablePort(PORTD);

    selectPinPushPullOutput(DSHOT0A);
    selectPinPushPullOutput(DSHOT0B);
    selectPinPushPullOutput(DSHOT1A);
    selectPinPushPullOutput(DSHOT1B);

    setPinAuxFunction(DSHOT0A, GPIO_PCTL_PC4_M0PWM6);
    setPinAuxFunction(DSHOT0B, GPIO_PCTL_PC5_M0PWM7);
    setPinAuxFunction(DSHOT1A, GPIO_PCTL_PD0_M1PWM0);
    setPinAuxFunction(DSHOT1B, GPIO_PCTL_PD1_M1PWM1);

    SYSCTL_RCGCPWM_R |= SYSCTL_RCGCPWM_R0 | SYSCTL_RCGCPWM_R1;
    _delay_cycles(3);

    SYSCTL_RCC_R &= ~SYSCTL_RCC_USEPWMDIV;

    SYSCTL_SRPWM_R = (SYSCTL_SRPWM_R0 | SYSCTL_SRPWM_R1);
    SYSCTL_SRPWM_R = 0;

    PWM0_3_CTL_R = 0;
    PWM1_0_CTL_R = 0;

    PWM0_3_GENA_R = (PWM_3_GENA_ACTLOAD_ZERO | PWM_3_GENA_ACTCMPAD_ONE);
    PWM0_3_GENB_R = (PWM_3_GENB_ACTLOAD_ZERO | PWM_3_GENB_ACTCMPBD_ONE);
    PWM1_0_GENA_R = (PWM_0_GENA_ACTLOAD_ZERO | PWM_0_GENA_ACTCMPAD_ONE);
    PWM1_0_GENB_R = (PWM_0_GENB_ACTLOAD_ZERO | PWM_0_GENB_ACTCMPBD_ONE);

    PWM0_3_LOAD_R = DSHOT_BIT_PERIOD_TICKS - 1U;
    PWM1_0_LOAD_R = DSHOT_BIT_PERIOD_TICKS - 1U;

    PWM0_3_CMPA_R = 0;
    PWM0_3_CMPB_R = 0;
    PWM1_0_CMPA_R = 0;
    PWM1_0_CMPB_R = 0;

    PWM0_3_CTL_R = PWM_0_CTL_ENABLE;
    PWM1_0_CTL_R = PWM_1_CTL_ENABLE;
    PWM0_SYNC_R = PWM_SYNC_SYNC3;
    PWM1_SYNC_R = PWM_SYNC_SYNC0;

    PWM0_ENABLE_R |= PWM_ENABLE_PWM6EN | PWM_ENABLE_PWM7EN;
    PWM1_ENABLE_R |= PWM_ENABLE_PWM0EN | PWM_ENABLE_PWM1EN;
}

void setThrottles(const uint16_t throttles[4]) {
    uint16_t frames[4] = {0};
    int8_t bit_idx;

    frames[0] = createFrame(throttles[0], 0);
    frames[1] = createFrame(throttles[1], 0);
    frames[2] = createFrame(throttles[2], 0);
    frames[3] = createFrame(throttles[3], 0);


    for (bit_idx = 15; bit_idx >= 0; --bit_idx) {
        uint32_t cmp_m1 = ((frames[0] >> bit_idx) & 0x01) ? DSHOT_T1H_TICKS : DSHOT_T0H_TICKS;
        uint32_t cmp_m2 = ((frames[1] >> bit_idx) & 0x01) ? DSHOT_T1H_TICKS : DSHOT_T0H_TICKS;
        uint32_t cmp_m3 = ((frames[2] >> bit_idx) & 0x01) ? DSHOT_T1H_TICKS : DSHOT_T0H_TICKS;
        uint32_t cmp_m4 = ((frames[3] >> bit_idx) & 0x01) ? DSHOT_T1H_TICKS : DSHOT_T0H_TICKS;

        while (!(PWM0_3_RIS_R & PWM_3_RIS_INTCNTZERO));
        PWM0_3_ISC_R = PWM_3_ISC_INTCNTZERO;

        PWM0_3_CMPA_R = cmp_m1;
        PWM0_3_CMPB_R = cmp_m2;
        PWM1_0_CMPA_R = cmp_m3;
        PWM1_0_CMPB_R = cmp_m4;
    }
    while (!(PWM0_3_RIS_R & PWM_3_RIS_INTCNTZERO));
    PWM0_3_ISC_R = PWM_3_ISC_INTCNTZERO;

    PWM0_3_CMPA_R = 0;
    PWM0_3_CMPB_R = 0;
    PWM1_0_CMPA_R = 0;
    PWM1_0_CMPB_R = 0;

}

