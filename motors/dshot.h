#ifndef DSHOT_H_
#define DSHOT_H_

#include <stdint.h>

#ifndef SYS_CLOCK_HZ
#define SYS_CLOCK_HZ 40000000U
#endif

#define DSHOT150  150U
#define DSHOT300  300U
#define DSHOT600  600U
#define DSHOT1200 1200U

#define ACTIVE_DSHOT_SPEED DSHOT300

#define DSHOT_BIT_PERIOD_TICKS (SYS_CLOCK_HZ / (ACTIVE_DSHOT_SPEED * 1000U))
#define DSHOT_T0H_TICKS        ((DSHOT_BIT_PERIOD_TICKS * 375U) / 1000U)
#define DSHOT_T1H_TICKS        ((DSHOT_BIT_PERIOD_TICKS * 750U) / 1000U)

void initDshot(void);
void setThrottles(const uint16_t pctp[4]);

#endif /* DSHOT_H_ */
