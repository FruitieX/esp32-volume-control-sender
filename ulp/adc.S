/* ULP Example: using ADC in deep sleep

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.

   This file contains assembly code which runs on the ULP.

   ULP wakes up to run this code at a certain period, determined by the values
   in SENS_ULP_CP_SLEEP_CYCx_REG registers. On each wake up, the program
   measures input voltage on the given ADC channel 'adc_oversampling_factor'
   times. Measurements are accumulated and average value is calculated.
   Average value is compared to the two thresholds: 'low_thr' and 'high_thr'.
   If the value is less than 'low_thr' or more than 'high_thr', ULP wakes up
   the chip from deep sleep.
*/

/* ULP assembly files are passed through C preprocessor first, so include directives
   and C macros may be used in these files 
 */
#include "soc/rtc_cntl_reg.h"
#include "soc/rtc_io_reg.h"
#include "soc/soc_ulp.h"

	/* ADC1 channel 5, GPIO33 */
	.set adc_channel, 5

	/* Configure the number of ADC samples to average on each measurement.
	   For convenience, make it a power of 2. */
	.set adc_oversampling_factor_log, 4
	.set adc_oversampling_factor, (1 << adc_oversampling_factor_log)

	/* How large a change in ADC value needs to be detected before we wake the SoC */
	.set diff_thr, 10

	/* Define variables, which go into .bss section (zero-initialized data) */
	.bss

	/* Counter of measurements done */
	.global sample_counter
sample_counter:
	.long 0

	.global result
result:
	.long 0

	.global last_result
last_result:
	.long 0

	/* Code goes into .text section */
	.text
	.global entry
entry:
	/* increment sample counter */
	move r3, sample_counter
	ld r2, r3, 0
	add r2, r2, 1
	st r2, r3, 0

	/* do measurements using ADC */
	/* r0 will be used as accumulator */
	move r0, 0

	/* initialize the loop counter */
	stage_rst
measure:
	/* set onboard led high */
	/*
	.set led_gpio, 12              // gpio pin 2 is rtc pin 12
	// might be needed for some pads, but surely not #2
	// WRITE_RTC_REG(RTC_IO_TOUCH_PAD2_REG, RTC_IO_TOUCH_PAD2_TO_GPIO_S, 1, 1)
	WRITE_RTC_REG(RTC_IO_TOUCH_PAD2_REG, RTC_IO_TOUCH_PAD2_MUX_SEL_S, 1, 1)
	WRITE_RTC_REG(RTC_GPIO_OUT_REG, RTC_GPIO_OUT_DATA_S + led_gpio, 1, 1)
	WRITE_RTC_REG(RTC_GPIO_ENABLE_W1TS_REG, RTC_GPIO_ENABLE_W1TS_S + led_gpio, 1, 1)
	*/

	// Is there a better way of finding the correct constants than staring at
	// https://i0.wp.com/randomnerdtutorials.com/wp-content/uploads/2018/08/esp32-pinout-chip-ESP-WROOM-32.png
	// and digging through header files?
	/* set pot pin high */
	.set pot_gpio, 17              // gpio pin 27 is rtc pin 17
	WRITE_RTC_REG(RTC_IO_TOUCH_PAD7_REG, RTC_IO_TOUCH_PAD7_MUX_SEL_S, 1, 1)
	WRITE_RTC_REG(RTC_GPIO_OUT_REG, RTC_GPIO_OUT_DATA_S + pot_gpio, 1, 1)
	WRITE_RTC_REG(RTC_GPIO_ENABLE_W1TS_REG, RTC_GPIO_ENABLE_W1TS_S + pot_gpio, 1, 1)

	/* measure and add value to accumulator */
	adc r1, 0, adc_channel + 1
	add r0, r0, r1

	/* increment loop counter and check exit condition */
	stage_inc 1
	jumps measure, adc_oversampling_factor, lt

	/* set onboard led low */
	// WRITE_RTC_REG(RTC_GPIO_ENABLE_W1TC_REG, RTC_GPIO_ENABLE_W1TC_S + led_gpio, 1, 1)

	/* set pot pin low */
	WRITE_RTC_REG(RTC_GPIO_ENABLE_W1TC_REG, RTC_GPIO_ENABLE_W1TC_S + pot_gpio, 1, 1)

	/* divide accumulator by adc_oversampling_factor.
	   Since it is chosen as a power of two, use right shift */
	rsh r0, r0, adc_oversampling_factor_log

	/* averaged value is now in r0 */

	/* store last_result in r2 */
	move r3, last_result
	ld r2, r3, 0

	/* store diff_thr in r1 */
	move r1, diff_thr

	/* begin: scale diff_thr (r1) for log pot */
	/* r3 = last_result (r2) >> 0x03 */
	/* rsh r3, r2, 0x05 */

	/* diff_thr (r1) = diff_thr (r1) + r3 */
	/* add r1, r1, r3 */
	/* end: scale diff_thr (r1) for log pot */

	/* low_thr (r3) = last_result (r2) - diff_thr (r1) */
	sub r3, r2, r1
	jump zero_low_thr, ov

check_low_thr:
	/* wake up if value (r0) < low_thr (r3) */
	sub r3, r0, r3
	jump wake_up, ov

	/* high_thr (r3) = last_result (r2) + diff_thr (r1) */
	add r3, r2, r1

	/* wake up if value (r0) > high_thr (r3) */
	sub r3, r3, r0
	jump wake_up, ov

	/* value within range, end the program */
	.global exit
exit:
	halt

zero_low_thr:
	move r3, 0
	jump check_low_thr

	.global wake_up
wake_up:
	/* store averaged value from r0 into result */
	move r3, result
	st r0, r3, 0

	/* Check if the system can be woken up */
	READ_RTC_FIELD(RTC_CNTL_LOW_POWER_ST_REG, RTC_CNTL_RDY_FOR_WAKEUP)
	and r0, r0, 1
	jump exit, eq

	/* Wake up the SoC, end program */
	wake
	WRITE_RTC_FIELD(RTC_CNTL_STATE0_REG, RTC_CNTL_ULP_CP_SLP_TIMER_EN, 0)
	halt
