#include <Snooze.h>

__attribute__ ((section(".dmabuffers"), used)) SLEEP_MODE Snooze::sleep_mode;
CLOCK_MODE Snooze::clock_mode;

extern "C" {
	/**
	 *  wakeupISR - handle LLWU interrupts after wakeup.
	 */
	void wakeup_isr( void ) {
		NVIC_DISABLE_IRQ( IRQ_LLWU ); // disable wakeup isr
		if ( Snooze::sleep_mode == LLS ) {
			__disable_irq( );
			llwuFlag = llwu_clear_flags( );// clear flags
			lptmrISR( );
			cmp0ISR( );
	#ifdef KINETISK
			rtcISR( );
	#endif
			tsiISR( );
			/************************************
			 * back to PEE if in PBE, else it is
			 * in either BLPI/BLPE, if so nothing
			 * to do.
			 ************************************/
			pbe_pee( );
			__enable_irq( );
		}
	}

	/*******************************************************************************
	 *
	 *       startup_early_hook -
	 *
	 *******************************************************************************/
	void startup_early_hook( ) {
		startup_default_early_hook();
		if ( PMC_REGSC & PMC_REGSC_ACKISO ) {
			llwuFlag = llwu_clear_flags( );// clear flags
			llwu_disable( );
		}
	}

} // extern "C""
