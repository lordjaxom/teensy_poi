#ifndef TLIBCPP_PINGROUP
#define TLIBCPP_PINGROUP

#include <stddef.h>

namespace tlibcpp {

	namespace detail {

		template< typename... T >
		struct pingroup_helper
		{
			static void output(bool value) __attribute__((always_inline))
			{
			}

			static void pullup() __attribute__((always_inline))
			{
			}

			static void pulldown() __attribute__((always_inline))
			{
			}

			static void set(bool value) __attribute__((always_inline))
			{
			}

			static void toggle() __attribute__((always_inline))
			{
			}
		};

		template< typename T0, typename... T >
		struct pingroup_helper< T0, T... >
		{
			static void output(bool value) __attribute__((always_inline))
			{
				T0::output(value);
				pingroup_helper< T... >::output(value);
			}

			static void pullup() __attribute__((always_inline))
			{
				T0::pullup();
				pingroup_helper< T... >::pullup();
			}

			static void pulldown() __attribute__((always_inline))
			{
				T0::pulldown();
				pingroup_helper< T... >::pulldown();
			}

			static void set(bool value) __attribute__((always_inline))
			{
				T0::set(value);
				pingroup_helper< T... >::set(value);
			}

			static void toggle() __attribute__((always_inline))
			{
				T0::toggle();
				pingroup_helper< T... >::toggle();
			}
		};

		struct pingroup_data
		{
			uint32_t volatile* set;
			uint32_t volatile* clear;
			uint32_t volatile* toggle;
			uint8_t bit;
			template< typename T > constexpr pingroup_data(T)
				: set(T::port_type::set)
				, clear(T::port_type::clear)
				, toggle(T::port_type::toggle)
				, bit(T::bit) {}
		};

	} // namespace detail

	template< typename... T >
	struct pingroup
	{
		static size_t const size = sizeof...(T);

		static void output(bool value = true) __attribute__((always_inline))
		{
			detail::pingroup_helper< T... >::output(value);
		}

		static void input() __attribute__((always_inline))
		{
			output(false);
		}

		static void pullup() __attribute__((always_inline))
		{
			detail::pingroup_helper< T... >::pullup();
		}

		static void pulldown() __attribute__((always_inline))
		{
			detail::pingroup_helper< T... >::pulldown();
		}

		static void set(bool value = true) __attribute__((always_inline))
		{
			detail::pingroup_helper< T... >::set(value);
		}

		static void clear() __attribute__((always_inline))
		{
			set(false);
		}

		static void toggle() __attribute__((always_inline))
		{
			detail::pingroup_helper< T... >::toggle();
		}

		static void set(size_t index, bool value = true) __attribute__((always_inline))
		{
			detail::pingroup_data const& port = ports[index];
			if (value) {
				*port.set = (1 << port.bit);
			}
			else {
				*port.clear = (1 << port.bit);
			}
		}

		static void clear(size_t index) __attribute__((always_inline))
		{
			set(index, false);
		}

		static void toggle(size_t index) __attribute__((always_inline))
		{
			detail::pingroup_data const& port = ports[index];
			*port.toggle = (1 << port.bit);
		}

		static constexpr detail::pingroup_data ports[] { detail::pingroup_data(T())... };
	};

	template< typename... T > constexpr detail::pingroup_data pingroup< T... >::ports[];

} // namespace tlibcpp

#endif // TLIBCPP_PINGROUP
