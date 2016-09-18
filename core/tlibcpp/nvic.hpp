#ifndef TLIBCPP_NVIC_HPP
#define TLIBCPP_NVIC_HPP

#include <teensy.h>

namespace tlibcpp {

template< size_t Vector >
struct nvic
{
	static size_t const vector = Vector;
	static size_t const irq = vector - 16;
	static size_t const register_offset = irq / 32;
	static size_t const register_bit = irq % 32;
	static size_t const priority_offset = irq;

	static void clear_pending() __attribute__((always_inline))
	{
		NVIC_ICPR(register_offset) |= (1 << register_bit);
	}

	static void enable(bool value = true) __attribute__((always_inline))
	{
		if (value) {
			NVIC_ISER(register_offset) |= (1 << register_bit);
		} 
		else {
			NVIC_ICER(register_offset) |= (1 << register_bit);
		}
	}

	static void disable() __attribute__((always_inline))
	{
		enable(false);
	}

	static void set_priority( uint8_t priority ) __attribute__((always_inline))
	{
		NVIC_IP(priority_offset) = (priority < 15 ? priority : 15) << 4;
	}
};

} // namespace tlibcpp

#endif // TLIBCPP_NVIC_HPP
