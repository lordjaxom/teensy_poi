#ifndef _teensy_func_h_
#define _teensy_func_h_

#include "teensy.h"

#ifdef __cplusplus
extern "C" {
#endif

extern volatile uint32_t systick_millis_count;

static inline uint32_t millis(void) __attribute__((always_inline));
static inline uint32_t millis(void)
{
	return systick_millis_count; // single aligned 32 bit is atomic
}

static inline void delayMicroseconds(uint32_t) __attribute__((always_inline));
static inline void delayMicroseconds(uint32_t usec)
{
#if F_CPU == 168000000
	uint32_t n = usec * 56;
#elif F_CPU == 144000000
	uint32_t n = usec * 48;
#elif F_CPU == 120000000
	uint32_t n = usec * 40;
#elif F_CPU == 96000000
	uint32_t n = usec << 5;
#elif F_CPU == 72000000
	uint32_t n = usec * 24;
#elif F_CPU == 48000000
	uint32_t n = usec << 4;
#elif F_CPU == 24000000
	uint32_t n = usec << 3;
#elif F_CPU == 16000000
	uint32_t n = usec << 2;
#elif F_CPU == 8000000
	uint32_t n = usec << 1;
#elif F_CPU == 4000000
	uint32_t n = usec;
#elif F_CPU == 2000000
	uint32_t n = usec >> 1;
#endif
    // changed because a delay of 1 micro Sec @ 2MHz will be 0
	if (n == 0) return;
	__asm__ volatile(
		"L_%=_delayMicroseconds:"		"\n\t"
#if F_CPU < 24000000
		"nop"					"\n\t"
#endif
#ifdef KINETISL
		"sub    %0, #1"				"\n\t"
#else
		"subs   %0, #1"				"\n\t"
#endif
		"bne    L_%=_delayMicroseconds"		"\n"
		: "+r" (n) :
	);
}

uint32_t micros(void);
void delay(uint32_t ms);

char * ultoa(unsigned long val, char *buf, int radix);
char * ltoa(long val, char *buf, int radix);
static inline char * utoa(unsigned int val, char *buf, int radix) __attribute__((always_inline, unused));
static inline char * utoa(unsigned int val, char *buf, int radix) { return ultoa(val, buf, radix); }
static inline char * itoa(int val, char *buf, int radix) __attribute__((always_inline, unused));
static inline char * itoa(int val, char *buf, int radix) { return ltoa(val, buf, radix); }

#ifdef __cplusplus
} // extern "C"
#endif

#endif /* _teensy_func_h_ */
