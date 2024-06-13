/*
 * Copyright (c) 2019, NVIDIA CORPORATION.  All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *  * Neither the name of NVIDIA CORPORATION nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
 * OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <timeserver.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include <printf-isr.h>
#include <tke-tegra.h>
#include <tke-tegra-hw.h>
#include <gpio-client.h>
#include "timer-app.h"


#include <gpio-provider.h>

/* gpio-aon.h has GPIO_APP* defines */
#include "gpio-aon.h"


/* Demo timer app which sets up 5 second periodic timer, modify below define to
 * adjust periodic value of the timer.
 */
#define TIMER2_PTV 1000000
#define TKE_TIMER_TMRCR_0_PTV		0x1fffffffU
#define TKE_TIMER_TMRCR_0		0x0
#define TKE_TIMER_TMRCR_0_PER		1 << 30
#define TKE_TIMER_TMRCR_0_EN		1 << 31

#define CODE_OFFSET_TICKS   125

 //static void tegra_tke_timer_writel(const struct tegra_tke_id *id, uint32_t val, uint32_t reg);


//Let's run the timer at 120 Hz for now

static void timer2_callback(void *data)
{
	uint64_t tsc_aligned = (tegra_tke_get_tsc64() + timeserverdata.phaseoffset_ticks) % timeserverdata.ticks_persecond;
	//Past here all numbers should be small wrapping around every second.
	uint32_t increment = timeserverdata.ticks_persecond/120;//120Hz for now
	uint32_t count = tsc_aligned/increment;
	//This will have a little bit of delay for now but should be ok and is easiest for now.
	if(timeserverdata.synced>0)
	{
		//PPS PIN
		if(count <= 11) {
            //printf_isr("ticks_persecond: %u, phaseoffset_ticks: %u\r\n", timeserverdata.ticks_persecond, timeserverdata.phaseoffset_ticks);
			gpio_set_value(GPIO_APP_IN, 1);
			//Let's not turn of the timer. Once it is synced once it should be good for a long time. 
			//--timeserverdata.synced;
        } else if(count > 11) {
			gpio_set_value(GPIO_APP_IN, 0);
        }
		//Thermal pin 120 hz. 50% duty cycle
		if(count%2 == 0)
			gpio_set_value(GPIO_APP_OUT, 1);
		else
			gpio_set_value(GPIO_APP_OUT, 0);
	}else
	{
		 if(count == 0) 
		 	printf_isr("No sync signal received from main computer. not outputting pulses\r\n");
	}
	//Increment in "increments." Subract any misalignment for the next timer deadline.
	uint32_t newcountergoal = increment - ( tsc_aligned - increment * count);
	// Don't setup a completely new timer. Just change the target count. Not sure why we need -1 but that was in the original setup code.
	uint32_t tmrcr = ((newcountergoal - 1) & TKE_TIMER_TMRCR_0_PTV) | TKE_TIMER_TMRCR_0_EN | TKE_TIMER_TMRCR_0_PER;
	tegra_tke_timer_writel(&tegra_tke_id_timer2, tmrcr, TKE_TIMER_TMRCR_0);
}

void timer_app_init(void)
{
	

	int val = gpio_direction_out(GPIO_APP_OUT, 0);
	if (val) {
		return;
	}
	val = gpio_direction_out(GPIO_APP_IN, 0);
	if (val) {
		return;
	}

	tegra_tke_set_up_timer(&tegra_tke_id_timer2, TEGRA_TKE_CLK_SRC_TSC_BIT0,
			       true, TIMER2_PTV, timer2_callback, 0);

	
}
