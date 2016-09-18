#ifndef TLIBCPP_NULL_PIN_HPP
#define TLIBCPP_NULL_PIN_HPP

namespace tlibcpp {

struct null_pin
{
	// direction functions

	static bool is_output() __attribute__((always_inline))
	{
		return false;
	}

	static void output(bool value = true) __attribute__((always_inline))
	{
	}

	static void input() __attribute__((always_inline))
	{
	}

	// output functions

	static void get() __attribute__((always_inline))
	{
	}

	static void clear() __attribute__((always_inline))
	{
	}

	static void set(bool value = true) __attribute__((always_inline))
	{
	}

	static void toggle() __attribute__((always_inline))
	{
	}

	// input functions

	static bool read() __attribute__((always_inline))
	{
		return false;
	}

	static void pullup() __attribute__((always_inline))
	{
	}

	static void pulldown() __attribute__((always_inline))
	{
	}
};

} // namespace tlibcpp

#endif // TLIBCPP_NULL_PIN_HPP