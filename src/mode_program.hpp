#ifndef TEENSY_POI_MODE_PROGRAM_HPP
#define TEENSY_POI_MODE_PROGRAM_HPP

#include "io_service.hpp"
#include "leds.hpp"
#include "led_algorithm.hpp"
#include "usb_serial.hpp"

namespace tp {

    template< typename Manager >
    class ModeProgram
    {
        static constexpr uint32_t usUntilInactive = 200000;
        static constexpr uint32_t usBetweenUpdates = 1000;
        static constexpr size_t bufferSize = 16;

        static_assert( flashBlockSize > bufferSize, "flashBlockSize must be greater than bufferSize" );
        static_assert( flashBlockSize % bufferSize == 0, "flashBlockSize must be a multiple of buffer size" );

        using IoServiceType = IoService< usBetweenUpdates, USBSerial >;
        using USBSerialType = USBSerial< IoServiceType >;

        using ReadyAnimation = LedFadeAnimation< 2000000, LedColor::BLACK, LedColor::BLUE >;
        using WorkingAnimation = LedFlashAnimation< 50000, 50000, LedColor::DIM_CYAN, LedColor::DIM_YELLOW >;
        using ChargingAnimation = LedFlashAnimation< 950000, 50000, LedColor::BLACK, LedColor::DIM_PURPLE >;
        using BatteryProgress = LedProgress< 6, 10, LedColor::FAINT_CYAN, LedColor::DIM_CYAN >;
        template< size_t Start > using VoltageProgress = LedProgress< Start, 10, LedColor::FAINT_PURPLE, LedColor::DIM_PURPLE >;

    public:
        explicit ModeProgram( Manager const& manager )
            : manager_( manager )
            , ioService_( manager_.stopwatch() )
            , usbSerial_( ioService_ )
            , active_()
            , inactiveCountdown_()
            , nextUpdateCountdown_()
        {
            usbSerial_.flushInput();
            beginCommand();
        }

        ModeProgram( ModeProgram const& ) = delete;

		void buttonClicked()
		{
		}

		void advance()
		{
		    ioService_.advance();
		    updateStatus();
		}

    private:
        void activate()
        {
            active_ = true;
            inactiveCountdown_ = usUntilInactive;
        }

        void updateStatus()
        {
            if ( active_ && manager_.stopwatch().countdown( inactiveCountdown_ ) == 0 ) {
                active_ = false;
            }

            if ( manager_.stopwatch().countdown( nextUpdateCountdown_ ) == 0 ) {
                auto&& pixels = leds_.pixels();
                auto timestamp = manager_.stopwatch().timestamp();

                pixels[ 0 ] = ReadyAnimation::animate( timestamp );
                pixels[ 1 ] = manager_.status().connected() ? LedColor::DIM_GREEN : LedColor::BLACK;
                pixels[ 2 ] = active_ ? WorkingAnimation::animate( timestamp ) : LedColor::BLACK;
                pixels[ 3 ] = manager_.status().charging() ? ChargingAnimation::animate( timestamp ) : LedColor::BLACK;

                BatteryProgress::percentage( pixels, manager_.status().percentage() );

#if 0
                auto voltage = manager_.status().voltage();
                ModeProgramDetail::VoltageProgress< 17 >::progress( pixels, voltage / 1000 );
                ModeProgramDetail::VoltageProgress< 28 >::progress( pixels, ( voltage / 100 ) % 10 );
#endif

                leds_.send();

                nextUpdateCountdown_ = usBetweenUpdates;
            }
        }

        void beginCommand()
        {
        	usbSerial_.asyncRead( buffer_, magicPacketLength + 1, libcpp__mem_fn( &ModeProgram::handleCommand, *this ) );
        }

        void handleCommand()
        {
            activate();

            auto p = std::begin( buffer_ );
            if ( std::equal( std::begin( magicPacket ), std::end( magicPacket ), p ) ) {
                p += magicPacketLength;
                switch ( *p ) {
                    case commandHello: {
                        sendResponse( responseOk, libcpp__mem_fn( &ModeProgram::beginCommand, *this ) );
                        return;
                    }
                    case commandRead: {
                        usbSerial_.asyncRead( buffer_, 2, libcpp__mem_fn( &ModeProgram::handleAddress< false >, *this ) );
                        return;
                    }
                    case commandWrite: {
                        usbSerial_.asyncRead( buffer_, 2, libcpp__mem_fn( &ModeProgram::handleAddress< true >, *this ) );
                        return;
                    }
                }
            }
            // usbSerial_.flushInput();
            beginCommand();
        }

        template< bool Write >
        void handleAddress()
        {
            activate();

            blockNo_ = ( buffer_[ 1 ] << 8 ) | buffer_[ 0 ];
            index_ = 0;
            sendResponse( responseAwaitData, Write
                         ? libcpp__mem_fn( &ModeProgram::beginEraseFlash, *this )
                         : libcpp__mem_fn( &ModeProgram::beginReadFlash, *this ) );
        }

        void beginReadFlash()
        {
            activate();

            uint32_t address = blockNo_ * flashBlockSize + index_ * bufferSize;
            flash_.readBytes( address, buffer_, bufferSize );
            usbSerial_.asyncWrite( buffer_, bufferSize, libcpp__mem_fn( &ModeProgram::handleFlashSent, *this ) );
        }

        void handleFlashSent()
        {
            activate();

            if ( ++index_ == flashBlockSize / bufferSize ) {
                sendResponse( responseOk, libcpp__mem_fn( &ModeProgram::beginCommand, *this ) );
            }
            else {
                beginReadFlash();
            }
        }

        void beginEraseFlash()
        {
            activate();

            uint32_t address = blockNo_ * flashBlockSize;
            flash_.eraseBlock( address );
            beginWriteFlash();
        }

        void beginWriteFlash()
        {
            activate();

            usbSerial_.asyncRead( buffer_, bufferSize, libcpp__mem_fn( &ModeProgram::handleFlashReceived, *this ) );
        }

        void handleFlashReceived()
        {
            activate();

            uint32_t address = blockNo_ * flashBlockSize + index_ * bufferSize;
            flash_.writeBytes( address, buffer_, bufferSize );
            ++index_;

            if ( index_ == flashBlockSize / bufferSize ) {
                sendResponse( responseOk, libcpp__mem_fn( &ModeProgram::beginCommand, *this ) );
            }
            else {
                beginWriteFlash();
            }
        }

        void sendResponse( uint8_t response, libcpp::function< void () > const& handler )
        {
            uint8_t* p = std::copy( std::begin( magicPacket ), std::end( magicPacket ), std::begin( buffer_ ) );
            *p = response;
            usbSerial_.asyncWrite( buffer_, magicPacketLength + 1, handler );
        }

        Manager const& manager_;
        IoServiceType ioService_;
        USBSerialType usbSerial_;
		typename Manager::Leds leds_;
		typename Manager::Flash flash_;
		bool active_;
		uint32_t inactiveCountdown_;
		uint32_t nextUpdateCountdown_;
		uint8_t buffer_[ bufferSize ];
		uint16_t blockNo_;
		uint16_t index_;
    };

} // namespace tp

#endif // TEENSY_POI_MODE_PROGRAM_HPP
