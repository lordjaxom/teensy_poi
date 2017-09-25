#ifndef TEENSY_POI_MANAGER_HPP
#define TEENSY_POI_MANAGER_HPP

#include <algorithm>
#include <iterator>
#include <new>
#include <type_traits>
#include <utility>

#include <libcpp/variant.hpp>

#include "stopwatch.hpp"

namespace tp {

	namespace ManagerDetail {

		struct ButtonClickedVisitor
		{
		    template< typename T >
		    void operator()( T& value )
		    {
		        value.buttonClicked();
		    }
		};


		struct AdvanceVisitor
		{
		    template< typename T >
		    void operator()( T& value )
		    {
		        value.advance();
		    }
		};

	} // namespace ManagerDetail


	template< typename TStatus, typename TButton, typename TLeds, typename TFlash, typename TMotion, template< typename > class... TModes >
	class Manager
	{
		using Modes = libcpp::variant< TModes< Manager >... >;

	public:
	    using Status = TStatus;
		using Button = TButton;
		using Leds = TLeds;
		using Flash = TFlash;
		using Motion = TMotion;

		static constexpr uint32_t tickUs = Button::debounceUs;

		Manager()
			: status_( stopwatch_ )
			, button_( stopwatch_ )
		{
			status_.onLowVoltage = libcpp__mem_fn( &Manager::statusLowVoltage, *this );
			button_.onClicked = libcpp__mem_fn( &Manager::buttonClicked, *this );
			button_.onLongpressed = libcpp__mem_fn( &Manager::buttonLongpressed, *this );

			next();
		}

		Manager( Manager const& ) = delete;

		void statusLowVoltage()
		{
			if ( modes_.index() != 0 ) {
				modes_.emplace( 0, *this );
			}
		}

		void buttonClicked()
		{
		    modes_.visit( ManagerDetail::ButtonClickedVisitor() );
		}

		void buttonLongpressed()
		{
			next();
		}

		void advance()
		{
			stopwatch_.advance();
			status_.advance();
			button_.advance();
			modes_.visit( ManagerDetail::AdvanceVisitor() );
		}

		Stopwatch const& stopwatch() const { return stopwatch_; }
		Status const& status() const { return status_; }
		Button const& button() const { return button_; }

	private:
		void next()
		{
		    modes_.emplace( modes_ ? ( modes_.index() + 1 ) % sizeof...( TModes ) : 0, *this );
		}

		Stopwatch stopwatch_;
		Status status_;
		Button button_;
		Modes modes_;
	};

} // namespace tp

#endif // TEENSY_POI_MANAGER_HPP
