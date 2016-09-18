#ifndef TLIBCPP_PIN_HPP
#define TLIBCPP_PIN_HPP

namespace tlibcpp {

template< typename Port, uint8_t Bit >
struct pin
{
	static constexpr uint8_t bit = Bit;

	using port_type = Port;
	using pin_type = typename port_type::template pinx< bit >;

	static constexpr auto pdor_reg = port_type::pdor_reg;
	static constexpr auto psor_reg = port_type::psor_reg;
	static constexpr auto pcor_reg = port_type::pcor_reg;
	static constexpr auto ptor_reg = port_type::ptor_reg;
	static constexpr auto pddr_reg = port_type::pddr_reg;
	static constexpr auto pdir_reg = port_type::pdir_reg;
	static constexpr auto pcr_reg = pin_type::pcr_reg;

	// direction functions

	static bool is_output() __attribute__((always_inline))
	{
		return *pddr_reg & (1 << bit);
	}

	static void output(bool value = true) __attribute__((always_inline))
	{
		if (value) {
			*pddr_reg |= (1 << bit);
			*pcr_reg = PORT_PCR_SRE | PORT_PCR_DSE | PORT_PCR_MUX( 1 );
		}
		else {
			*pddr_reg &= ~(1 << bit);
			*pcr_reg = PORT_PCR_MUX( 1 );
		}
	}

	static void input() __attribute__((always_inline))
	{
		output(false);
	}

	// output functions

	static void get() __attribute__((always_inline))
	{
		return *pdor_reg & (1 << bit);
	}

	static void clear() __attribute__((always_inline))
	{
		set(false);
	}

	static void set(bool value = true) __attribute__((always_inline))
	{
		if (value) {
			*psor_reg = (1 << bit);
		}
		else {
			*pcor_reg = (1 << bit);
		}
	}

	static void toggle() __attribute__((always_inline))
	{
		*ptor_reg = (1 << bit);
	}

	// input functions

	static bool read() __attribute__((always_inline))
	{
		return *pdir_reg & (1 << bit);
	}

	static void pullup() __attribute__((always_inline))
	{
		*pin_type::pcr_reg = PORT_PCR_MUX( 1 ) | PORT_PCR_PE | PORT_PCR_PS;
	}

	static void pulldown() __attribute__((always_inline))
	{
		*pin_type::pcr_reg = PORT_PCR_MUX( 1 ) | PORT_PCR_PE;
	}
};

template< typename Port, uint8_t Bit >
struct inv_pin : private pin< Port, Bit >
{
	typedef pin< Port, Bit > base_type;

	using base_type::port_type;
	using base_type::bit;

	using base_type::pdor_reg;
	using base_type::psor_reg;
	using base_type::pcor_reg;
	using base_type::ptor_reg;
	using base_type::pddr_reg;
	using base_type::pdir_reg;
	using base_type::pcr_reg;

	// direction functions

	using base_type::is_output;
	using base_type::output;
	using base_type::input;

	// output functions

	static bool get() __attribute__((always_inline))
	{
		return !base_type::get();
	}

	static void clear() __attribute__((always_inline))
	{
		base_type::set();
	}

	static void set( bool value = true ) __attribute__((always_inline))
	{
		base_type::set( !value );
	}

	using base_type::toggle;

	// input functions

	static bool read() __attribute__((always_inline))
	{
		return !base_type::read();
	}

	using base_type::pullup;
	using base_type::pulldown;
};

} // namespace tlibcpp

#endif // TLIBCPP_PIN_HPP
