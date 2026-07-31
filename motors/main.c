/* INCLUDES */

//C-Std Lib.
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

//TM4C-Core
#include "tm4c123gh6pm.h"
#include "clock.h"
#include "wait.h"
#include "nvic.h"

//TM4C-Peripheral drivers
#include "gpio.h"
#include "uart0.h"

//Higher Level Peripheral Access
#define SHELL_IMPLEMENTATION
#include "shell.h"
#include "dshot.h"

/* GLOBALS CONSTS AND MACROS */
#define LED_G           PORTF,3
#define BUTTON          PORTF,4
//
#define M1              PORTC,4     //M0PWM6 : M0 PWM3 GENA   (DSHOT0A)
#define M2              PORTC,5     //M0PWM7 : M0 PWM3 GENB   (DSHOT0B)
#define M3              PORTD,0     //M1PWM0 : M0 PWM0 GENA   (DSHOT1A)
#define M4              PORTD,1     //M1PWM1 : M0 PWM0 GENB   (DSHOT1B)

#define JLX             PORTE,5
#define JLY             PORTE,4
#define JRX             PORTE,1
#define JRY             PORTE,2

//#define PWM_DIV         SYSCTL_RCC_PWMDIV_2
//#define PWM_LOAD        50000-1     //2.5ms period (400Hz)
//#define PWM_MIN         20000       //cmp min for 1000us
//#define PWM_MAX         40000       //cmp max for 2000us


//#define PWM_DIV         SYSCTL_RCC_PWMDIV_2
//#define PWM_LOAD        50000-1     //20ms period (50Hz)
//#define PWM_MIN         2500       //cmp min for 1000us
//#define PWM_MAX         5000       //cmp max for 2000us

#define PWM_DIV         0
#define PWM_LOAD        0
#define PWM_MIN         48
#define PWM_MAX         2047


#define PWM_RANGE       ((PWM_MAX)-(PWM_MIN))

/* SUB ROUTINE PROTOTYPES */
void initHw(void);

void initAdc(void);
void readAdc0Ss0(int16_t* data);

//Dshot impl
void _arm_seq(void);
void _disarm_seq(void);

//PWM impl
void initPwms(void);
void mix(int16_t* raws, uint16_t *mixed);
void setPwms(uint16_t *pwms);

/* MAIN ROUTINE */
int main(void)
{
    initHw();

    char buffer[50];
    int16_t raws[4];
    uint16_t pwms[4] = {0};
    bool armed = false;

    putsUart0(CLEAR_SCREEN);

    //go into joystick reading
    for(;;) {
        if(!armed) {
            while(getPinValue(BUTTON));
            _arm_seq();
            armed = true;
        } else {//armed
            if(!getPinValue(BUTTON)) {
                _disarm_seq();
                armed = false;
                continue;
            }
        }

        readAdc0Ss0(raws);
        mix(raws, pwms);


        if(0) setPwms(pwms);
        else  setThrottles(pwms);

        putsUart0(SAVE_POS);
        usprintf(buffer, "M1:%d   \nM2:%d   \nM3:%d   \nM4:%d   ", pwms[0], pwms[1], pwms[2], pwms[3]);
        putsUart0(buffer);
        putsUart0(RETURN_2_POS);
        waitMicrosecond(1e3);
    }
}

void initHw(void) {
    initSystemClockTo40Mhz();

    //Status LED
    enablePort(PORTF);
    selectPinPushPullOutput(LED_G);
    selectPinDigitalInput(BUTTON);
    enablePinPullup(BUTTON);

//    initPwms();
    initDshot();
    initAdc();

    //Shell
    initUart0();
    setUart0BaudRate(115200, 40e6);

    //Startup Sequence
    putsUart0(CLEAR_SCREEN);
    putsUart0(GOTO_HOME);
    putsUart0(HIDE_CURSOR);
    Print("Motor");
    setPinValue(LED_G, 1);
    waitMicrosecond(500e3);
    setPinValue(LED_G, 0);
}

void initAdc(void) {
    //Joystick Analogs
        enablePort(PORTE);
        SYSCTL_RCGCADC_R |= SYSCTL_RCGCADC_R0;
        selectPinAnalogInput(JLX);
        selectPinAnalogInput(JLY);
        selectPinAnalogInput(JRX);
        selectPinAnalogInput(JRY);
        setPinAuxFunction(JLX, GPIO_PCTL_PE5_AIN8);
        setPinAuxFunction(JLY, GPIO_PCTL_PE4_AIN9);
        setPinAuxFunction(JRX, GPIO_PCTL_PE1_AIN2);
        setPinAuxFunction(JRY, GPIO_PCTL_PE2_AIN1);

        //Disable ss0 for programming
        ADC0_ACTSS_R &= ~ ADC_ACTSS_ASEN0;
        //Use PLL Clock
        ADC0_CC_R = ADC_CC_CS_SYSPLL;
        //1Msps sample rate
        ADC0_PC_R = ADC_PC_SR_1M;
        ADC0_EMUX_R = ADC_EMUX_EM0_PROCESSOR;
        //Set HW sampling
        ADC0_SAC_R = ADC_SAC_AVG_16X;
#define ADC_CTL_DITHER 0x00000040
        ADC0_CTL_R |= ADC_CTL_DITHER;
#undef ADC_CTL_DITHER
        //Set the sample sequence to read all pins in a burst
        ADC0_SSMUX0_R = (2 << ADC_SSMUX0_MUX0_S) |  // PE1 AIN2 => JRX
                        (1 << ADC_SSMUX0_MUX1_S) |  // PE2 AIN1 => JRY
                        (9 << ADC_SSMUX0_MUX3_S) |  // PE4 AIN9 => JLX
                        (8 << ADC_SSMUX0_MUX2_S);   // PE5 AIN8 => JLY
        //Mark fourth sample as the end (and trigger interrupt used for dma)
        ADC0_SSCTL0_R = ADC_SSCTL0_END3 | ADC_SSCTL0_IE3;
        ADC0_ACTSS_R |= ADC_ACTSS_ASEN0;
}

void readAdc0Ss0(int16_t* data)
{
    ADC0_PSSI_R |= ADC_PSSI_SS0;                    // set start bit
    uint8_t i = 0;
    while (ADC0_ACTSS_R & ADC_ACTSS_BUSY);          // wait until SS3 is not busy
    while (ADC0_SSFSTAT0_R & ADC_SSFSTAT0_EMPTY);
    for(;i < 4; ++i) {
        data[i] = (int16_t)ADC0_SSFIFO0_R;          // get single result from the FIFO
    }
}

//=======================================================================================
//=======================================================================================




//=======================================================================================
//=======================================================================================
//only works on one of my ESC's
void initPwms() {
    enablePort(PORTC);
    enablePort(PORTD);

    //enable pins
    selectPinPushPullOutput(M1);
    selectPinPushPullOutput(M2);
    selectPinPushPullOutput(M3);
    selectPinPushPullOutput(M4);

    //set to use pwm
    setPinAuxFunction(M1, GPIO_PCTL_PC4_M0PWM6);
    setPinAuxFunction(M2, GPIO_PCTL_PC5_M0PWM7);
    setPinAuxFunction(M3, GPIO_PCTL_PD0_M1PWM0);
    setPinAuxFunction(M4, GPIO_PCTL_PD1_M1PWM1);

    //PWM (Motors)
    //MOTOR_FRONT_LEFT  (M1)    PC4 M0PWM6 : M0 PWM3 GENA
    //MOTOR_FRONT_RIGHT (M2)    PC5 M0PWM7 : M0 PWM3 GENB
    //MOTOR_BACK_LEFT   (M3)    PD0 M1PWM0 : M0 PWM0 GENA
    //MOTOR_BACK_RIGHT  (M4)    PD1 M1PWM1 : M0 PWM0 GENB

    SYSCTL_RCGCPWM_R |= SYSCTL_RCGCPWM_R0 | SYSCTL_RCGCPWM_R1;
    SYSCTL_RCC_R |= SYSCTL_RCC_USEPWMDIV | PWM_DIV;
    _delay_cycles(3);

    //Reset
    SYSCTL_SRPWM_R = SYSCTL_SRPWM_R1 | SYSCTL_SRPWM_R0;
    SYSCTL_SRPWM_R = 0;

    //Turn off
    PWM0_3_CTL_R = 0;
    PWM1_0_CTL_R = 0;

    //Configure
    PWM0_3_GENA_R = PWM_3_GENA_ACTCMPAD_ONE | PWM_3_GENA_ACTLOAD_ZERO;
    PWM0_3_GENB_R = PWM_3_GENB_ACTCMPBD_ONE | PWM_3_GENB_ACTLOAD_ZERO;

    PWM1_0_GENA_R = PWM_0_GENA_ACTCMPAD_ONE | PWM_0_GENA_ACTLOAD_ZERO;
    PWM1_0_GENB_R = PWM_0_GENB_ACTCMPBD_ONE | PWM_0_GENB_ACTLOAD_ZERO;

    PWM0_3_LOAD_R = PWM_LOAD;
    PWM1_0_LOAD_R = PWM_LOAD;

    PWM0_3_CMPA_R = 0;
    PWM0_3_CMPB_R = 0;
    PWM1_0_CMPA_R = 0;
    PWM1_0_CMPB_R = 0;

    //Turn on Generators
    PWM0_3_CTL_R = PWM_0_CTL_ENABLE;
    PWM1_0_CTL_R = PWM_1_CTL_ENABLE;

    PWM0_SYNC_R = PWM_SYNC_SYNC3;
    PWM1_SYNC_R = PWM_SYNC_SYNC0;

    //Enable Outputs
    PWM0_ENABLE_R |= PWM_ENABLE_PWM6EN | PWM_ENABLE_PWM7EN;
    PWM1_ENABLE_R |= PWM_ENABLE_PWM0EN | PWM_ENABLE_PWM1EN;
}

#define min(a,b) ((a)>(b)?(b):(a))
#define max(a,b) ((a)<(b)?(b):(a))
void mix(int16_t* raws, uint16_t *mixed){
    //raws[0] -> RY -> Ritch
    //raws[1] -> RX -> Roll
    //raws[2] -> LY -> Throttle
    //raws[3] -> Lx -> Yaw
    uint8_t i;
    int32_t norms[4];

    enum {
        THROTTLE=2,
        YAW=3,
        PITCH=0,
        ROLL=1
    };

    for(i = 0; i < 4; ++i) { norms[i] = (4096 - raws[i]) * 100 / 4096; }
    norms[PITCH] -= 50;
    norms[ROLL] -= 50;
    norms[YAW] -= 50;



    //mix
    mixed[0] = max(0, min(norms[THROTTLE] + norms[PITCH] + norms[ROLL] + norms[YAW], 100));
    mixed[1] = max(0, min(norms[THROTTLE] + norms[PITCH] - norms[ROLL] - norms[YAW], 100));
    mixed[2] = max(0, min(norms[THROTTLE] - norms[PITCH] + norms[ROLL] - norms[YAW], 100));
    mixed[3] = max(0, min(norms[THROTTLE] - norms[PITCH] - norms[ROLL] + norms[YAW], 100));

    for(i = 0; i < 4; ++i)
        mixed[i] = PWM_MIN + ((int32_t) mixed[i] * PWM_RANGE / 100);

}
#undef min
#undef max

void setPwms(uint16_t *pwms) {
    PWM0_3_CMPA_R = pwms[0];
    PWM0_3_CMPB_R = pwms[1];
    PWM1_0_CMPA_R = pwms[2];
    PWM1_0_CMPB_R = pwms[3];
}

void _arm_seq(void) {
    const uint16_t zeros[4] = {0,0,0,0};
    uint16_t i;
    for(i = 0; i < 500; ++i) {
        setThrottles(zeros);
        waitMicrosecond(2e3);   //2ms
    }
}

void _disarm_seq(void) {
    const uint16_t zeros[4] = {0,0,0,0};
    setThrottles(zeros);
    waitMicrosecond(500e3);
}

