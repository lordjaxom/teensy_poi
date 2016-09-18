#ifndef TEENSY_POI_MOTION_HPP
#define TEENSY_POI_MOTION_HPP

#include <math.h>
#include <algorithm>
#include <iterator>
#include <limits>

#include <I2C.h>
#include <MPU6050.h>

#include "power.hpp"

#define USE_DMP

namespace tp {

	template< typename Sda, typename Scl, bool Pullup, typename PowerPin >
    class Motion
    {
		using I2CType = I2CMaster< Sda, Scl, Pullup, 2000000, I2CMasterMode::IMM >;
		using MPUType = MPU6050< I2CType >;

		static_assert( std::numeric_limits< float >::has_infinity, "float must have infinity value" );

		static constexpr size_t staticBufferSize = 64;
		static constexpr float infinity = std::numeric_limits< float >::infinity();

		static constexpr size_t usBetweenDebug = 50000;

		static PowerGuard< PowerPin > power;

    public:
        explicit Motion( Stopwatch const& stopwatch )
            : stopwatch_( stopwatch )
            , mpu_( i2c_ )
            , packetSize_()
            , roundsPerSecond_()
            , ok_()
        {
            power.enable();
            delay( 250 );

#ifdef USE_DMP
            if ( mpu_.dmpInitialize() != 0 ) {
                return;
            }
#else
            mpu_.initialize();
            if ( !mpu_.testConnection() ) {
                return;
            }
#endif

            // values according to MPU6050_calibrate.ino
            mpu_.setXAccelOffset( -1503 );
            mpu_.setYAccelOffset( 774 );
            mpu_.setZAccelOffset( 1391 );
            mpu_.setXGyroOffset( 69 );
            mpu_.setYGyroOffset( -38 );
            mpu_.setZGyroOffset( 7 );

#ifdef USE_DMP
            mpu_.setDMPEnabled(true);
            packetSize_ = mpu_.dmpGetFIFOPacketSize();
            if ( packetSize_ > staticBufferSize ) {
				return;
            }
#else
            mpu_.setFullScaleGyroRange( MPU6050_GYRO_FS_2000 );
#endif

            ok_ = true;
        }

        Motion( Motion const& ) = delete;

        ~Motion()
        {
            mpu_.reset();
            delay( 50 );
            mpu_.setSleepEnabled( true );

            power.disable();
        }

        operator bool() const { return ok_; }

        float roundsPerSecond()
        {
            if ( roundsPerSecond_ == infinity ) {
                roundsPerSecond_ = std::sqrt( (float) ( gyro_[ 0 ] * gyro_[ 0 ] + gyro_[ 2 ] * gyro_[ 2 ] ) ) / 250.f;
            }
            return roundsPerSecond_;
        }

        void advance()
        {
#ifdef USE_DMP
            auto count = mpu_.getFIFOCount();
			if ( count >= 1024 ) {
				mpu_.resetFIFO();
			}
			else if ( count >= packetSize_ ) {
                uint8_t buffer[ staticBufferSize ];
                while ( count >= packetSize_ ) {
                    mpu_.getFIFOBytes( buffer, packetSize_ );
                    count -= packetSize_;
                }

                mpu_.dmpGetGyro( &gyro_[ 0 ], buffer );
                roundsPerSecond_ = infinity;
			}
#else
			mpu_.getRotation( &gyro_[ 0 ], &gyro_[ 1 ], &gyro_[ 2 ] );
			roundsPerSecond_ = infinity;
#endif

#if 0
			if ( debugCountdown_ == 0 ) {
				float xRot = gyro_[ 0 ] / 2000.0f;
				float yRot = gyro_[ 1 ] / 2000.0f;
				float zRot = gyro_[ 2 ] / 2000.0f;
				debug( xRot, " ", yRot, " ", zRot );
				//debug( gyro_[ 0 ], " ", gyro_[ 1 ], " ", gyro_[ 2 ] );
				debugCountdown_ = usBetweenDebug;
			}
#endif
        }

    private:
    	Stopwatch const& stopwatch_;
        I2CType i2c_;
        MPUType mpu_;
        uint16_t packetSize_;
        int16_t gyro_[ 3 ];
        float roundsPerSecond_;
        bool ok_;
    };

   	template< typename Sda, typename Scl, bool Pullup, typename PowerPin >
    PowerGuard< PowerPin > Motion< Sda, Scl, Pullup, PowerPin >::power;

} // namespace tp

#endif // TEENSY_POI_MOTION_HPP
