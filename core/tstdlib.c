#include "tstdlib.h"

uint32_t micros(void)
{
	uint32_t count, current, istatus;

	__disable_irq();
	current = SYST_CVR;
	count = systick_millis_count;
	istatus = SCB_ICSR;	// bit 26 indicates if systick exception pending
	__enable_irq();
	 //systick_current = current;
	 //systick_count = count;
	 //systick_istatus = istatus & SCB_ICSR_PENDSTSET ? 1 : 0;
	if ((istatus & SCB_ICSR_PENDSTSET) && current > 50) count++;
	current = ((F_CPU / 1000) - 1) - current;
#if defined(KINETISL) && F_CPU == 48000000
	return count * 1000 + ((current * (uint32_t)87381) >> 22);
#elif defined(KINETISL) && F_CPU == 24000000
	return count * 1000 + ((current * (uint32_t)174763) >> 22);
#endif
	return count * 1000 + current / (F_CPU / 1000000);
}

void delay(uint32_t ms)
{
    uint32_t start = micros();

	if (ms > 0) {
		while (1) {
			if ((micros() - start) >= 1000) {
				ms--;
				if (ms == 0) return;
				start += 1000;
			}
		}
	}
}
