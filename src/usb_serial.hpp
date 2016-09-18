#ifndef TEENSY_POI_USB_SERIAL_HPP
#define TEENSY_POI_USB_SERIAL_HPP

#include <stdint.h>

#include <usb_serial.h>
#include <libcpp/function.hpp>
#include <peripheral.h>

#include "io_service.hpp"

namespace tp {

    namespace USBSerialDetail {

        class AsyncRead
        {
        public:
            AsyncRead( uint8_t* buffer, uint16_t size, libcpp::function< void () > const& handler )
				: buffer_( buffer )
				, size_( size )
				, handler_( handler )
			{
			}

            bool operator()( uint32_t timeout )
            {
            	int available = usb_serial_available();
            	if ( available > 0 ) {
					uint32_t count = available < size_ ? available : size_;
					usb_serial_read( buffer_, count );
					buffer_ += count;
					size_ -= count;
                    if ( size_ == 0 ) {
                        handler_();
                        return true;
                    }
            	}
            	return false;
            }

		private:
			uint8_t* buffer_;
			uint16_t size_;
			uint16_t remain_;
			libcpp::function< void () > handler_;
        };

        class AsyncWrite
        {
        public:
            AsyncWrite( uint8_t const* buffer, uint16_t size, libcpp::function< void () > const& handler )
				: buffer_( buffer )
				, size_( size )
				, handler_( handler )
			{
			}

            bool operator()( uint32_t timeout )
            {
            	int available = usb_serial_write_buffer_free();
            	if ( available > 0 ) {
					uint32_t count = available < size_ ? available : size_;
					usb_serial_write( buffer_, count );
					buffer_ += count;
					size_ -= count;
                    if ( size_ == 0 ) {
                        handler_();
                        return true;
                    }
            	}
            	return false;
            }

		private:
			uint8_t const* buffer_;
			uint16_t size_;
			libcpp::function< void () > handler_;
        };

    } // namespace USBSerialDetail

    template< typename IoService >
    class USBSerial
        : public IoChannel<
            USBSerialDetail::AsyncRead,
            USBSerialDetail::AsyncWrite
        >
    {
    public:
    	explicit USBSerial( IoService& ioService )
			: ioService_( ioService )
		{
			usbEnable();
		}

        USBSerial( USBSerial const& ) = delete;

        ~USBSerial()
        {
        	usbDisable();
        }

        void asyncRead( uint8_t* buffer, uint16_t size, libcpp::function< void () > const& handler )
        {
			ioService_.invoke( USBSerialDetail::AsyncRead( buffer, size, handler ) );
        }

        void asyncWrite( uint8_t const* buffer, uint16_t size, libcpp::function< void () > const& handler )
        {
        	ioService_.invoke( USBSerialDetail::AsyncWrite( buffer, size, handler ) );
        }

        void flushInput()
        {
            usb_serial_flush_input();
        }

	private:
		IoService& ioService_;
    };

} // namespace tp

#endif // TEENSY_POI_USB_SERIAL_HPP
