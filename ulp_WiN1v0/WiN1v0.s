/*
   This file contains assembly code which runs on the ULP.

   ULP wakes up to run this code at a certain period, determined by the values
   in SENS_ULP_CP_SLEEP_CYCx_REG registers. On each wake up, the program
   checks the logic level of the RTC_GPIO selected, then increments the value
   of the tip_counter if the reading matches the desired level.
*/

/* ULP assembly files are passed through C preprocessor first, so include directives
   and C macros may be used in these files */
#include "soc/rtc_cntl_reg.h"
#include "soc/rtc_io_reg.h"
#include "soc/soc_ulp.h"

/* Define constants */
.set tip_trigger, 0x05
.set rtc_gpio_trigger_level, 1      /* 1: HIGH level, 0: LOW level */

/* Define variables, which go into .bss section (zero-initialized data) */
.bss

/* Counter of measurements done */
.global sample_counter
sample_counter:
.long 0

.global tip_counter
tip_counter:
.long 0

/* Code goes into .text section */
.text
.global entry
entry:

/* Increment sample counter */
move r3, sample_counter
ld R2, r3, 0
add R2, R2, 1
st R2, r3, 0

/* Initialize the loop counter */
stage_rst

measure:
/* Increment loop counter and check the RTC_GPIO condition */
stage_inc 1
READ_RTC_REG(RTC_GPIO_IN_REG,RTC_GPIO_IN_NEXT_S+12, rtc_gpio_trigger_level)
JUMPR tip_count,1,EQ

/*
move r2,tip_counter
sub r1,r2,tip_trigger
jump reset_count, EQ*/

jump exit

/* Increment tip_ounter */
tip_count:
move r3, tip_counter
ld R2, r3, 0
add R2, R2, 1
st R2, r3, 0
jump exit

/*
reset_count:
move r1,tip_counter
and r1,r1,0x00
jump wake_up
*/

/* End the program */
.global exit
exit:
/* Set the GPIO13 output LOW (clear output) to signal that ULP is now going down */
/* WRITE_RTC_REG(RTC_GPIO_OUT_W1TC_REG, RTC_GPIO_OUT_DATA_W1TC_S + 14, 1, 1) */
/* Enable hold on GPIO13 output */
/* WRITE_RTC_REG(RTC_IO_TOUCH_PAD4_REG, RTC_IO_TOUCH_PAD4_HOLD_S, 1, 1) */
halt

.global wake_up
wake_up:
/* Check if the system can be woken up */
READ_RTC_FIELD(RTC_CNTL_LOW_POWER_ST_REG, RTC_CNTL_RDY_FOR_WAKEUP)
and r0, r0, 1
jump exit, eq

/* Wake up the SoC, end program */
wake
WRITE_RTC_FIELD(RTC_CNTL_STATE0_REG, RTC_CNTL_ULP_CP_SLP_TIMER_EN, 0)
jump exit
