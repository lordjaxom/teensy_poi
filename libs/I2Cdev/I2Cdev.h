// I2Cdev library collection - Main I2C device class header file
// Abstracts bit and byte I2C R/W functions into a convenient class
// 2013-06-05 by Jeff Rowberg <jeff@rowberg.net>
//
// Changelog:
//      2015-10-30 - simondlevy : support i2c_t3 for Teensy3.1
//      2013-05-06 - add Francesco Ferrara's Fastwire v0.24 implementation with small modifications
//      2013-05-05 - fix issue with writing bit values to words (Sasquatch/Farzanegan)
//      2012-06-09 - fix major issue with reading > 32 bytes at a time with Arduino Wire
//                 - add compiler warnings when using outdated or IDE or limited I2Cdev implementation
//      2011-11-01 - fix write*Bits mask calculation (thanks sasquatch @ Arduino forums)
//      2011-10-03 - added automatic Arduino version detection for ease of use
//      2011-10-02 - added Gene Knight's NBWire TwoWire class implementation with small modifications
//      2011-08-31 - added support for Arduino 1.0 Wire library (methods are different from 0.x)
//      2011-08-03 - added optional timeout parameter to read* methods to easily change from default
//      2011-08-02 - added support for 16-bit registers
//                 - fixed incorrect Doxygen comments on some methods
//                 - added timeout value for read operations (thanks mem @ Arduino forums)
//      2011-07-30 - changed read/write function structures to return success or byte counts
//                 - made all methods static for multi-device memory savings
//      2011-07-28 - initial release

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2013 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

#ifndef _I2CDEV_H_
#define _I2CDEV_H_

#include <stddef.h>
#include <stdint.h>
#include <tstdlib.h>

// 1000ms default read timeout (modify with "I2CDEV_DEFAULT_READ_TIMEOUT = [ms];")
#define I2CDEV_DEFAULT_READ_TIMEOUT     1000

#define BUFFER_LENGTH 32

template< typename I2c >
class I2Cdev {
    public:
        I2Cdev(I2c& wire)
            : wire_(wire)
        {
        }

        int8_t readBit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t *data, uint16_t timeout=I2CDEV_DEFAULT_READ_TIMEOUT) {
            uint8_t b;
            uint8_t count = readByte(devAddr, regAddr, &b, timeout);
            *data = b & (1 << bitNum);
            return count;
        }
        int8_t readBitW(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint16_t *data, uint16_t timeout=I2CDEV_DEFAULT_READ_TIMEOUT) {
            uint16_t b;
            uint8_t count = readWord(devAddr, regAddr, &b, timeout);
            *data = b & (1 << bitNum);
            return count;
        }
        int8_t readBits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *data, uint16_t timeout=I2CDEV_DEFAULT_READ_TIMEOUT) {
            // 01101001 read byte
            // 76543210 bit numbers
            //    xxx   args: bitStart=4, length=3
            //    010   masked
            //   -> 010 shifted
            uint8_t count, b;
            if ((count = readByte(devAddr, regAddr, &b, timeout)) != 0) {
                uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
                b &= mask;
                b >>= (bitStart - length + 1);
                *data = b;
            }
            return count;
        }
        int8_t readBitsW(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint16_t *data, uint16_t timeout=I2CDEV_DEFAULT_READ_TIMEOUT) {
            // 1101011001101001 read byte
            // fedcba9876543210 bit numbers
            //    xxx           args: bitStart=12, length=3
            //    010           masked
            //           -> 010 shifted
            uint8_t count;
            uint16_t w;
            if ((count = readWord(devAddr, regAddr, &w, timeout)) != 0) {
                uint16_t mask = ((1 << length) - 1) << (bitStart - length + 1);
                w &= mask;
                w >>= (bitStart - length + 1);
                *data = w;
            }
            return count;
        }
        int8_t readByte(uint8_t devAddr, uint8_t regAddr, uint8_t *data, uint16_t timeout=I2CDEV_DEFAULT_READ_TIMEOUT) {
            return readBytes(devAddr, regAddr, 1, data, timeout);
        }
        int8_t readWord(uint8_t devAddr, uint8_t regAddr, uint16_t *data, uint16_t timeout=I2CDEV_DEFAULT_READ_TIMEOUT) {
            return readWords(devAddr, regAddr, 1, data, timeout);
        }
        int8_t readBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data, uint16_t timeout=I2CDEV_DEFAULT_READ_TIMEOUT) {
            #ifdef I2CDEV_SERIAL_DEBUG
                Serial.print("I2C (0x");
                Serial.print(devAddr, HEX);
                Serial.print(") reading ");
                Serial.print(length, DEC);
                Serial.print(" bytes from 0x");
                Serial.print(regAddr, HEX);
                Serial.print("...");
            #endif

            int8_t count = 0;
            uint32_t t1 = millis();

            // Arduino v1.0.1+, Wire library
            // Adds official support for repeated start condition, yay!

            // I2C/TWI subsystem uses internal buffer that breaks with large data requests
            // so if user requests more than BUFFER_LENGTH bytes, we have to do it in
            // smaller chunks instead of all at once
            for (uint8_t k = 0; k < length; k += length < BUFFER_LENGTH ? length : BUFFER_LENGTH) {
                wire_.beginTransmission(devAddr);
                wire_.write(regAddr);
                wire_.endTransmission();
                wire_.beginTransmission(devAddr);
                wire_.requestFrom(devAddr, (uint8_t)(length - k < BUFFER_LENGTH ? length - k : BUFFER_LENGTH));

                for (; wire_.available() && (timeout == 0 || millis() - t1 < timeout); count++) {
                    data[count] = wire_.read();
                    #ifdef I2CDEV_SERIAL_DEBUG
                        Serial.print(data[count], HEX);
                        if (count + 1 < length) Serial.print(" ");
                    #endif
                }
            }

            // check for timeout
            if (timeout > 0 && millis() - t1 >= timeout && count < length) count = -1; // timeout

            #ifdef I2CDEV_SERIAL_DEBUG
                Serial.print(". Done (");
                Serial.print(count, DEC);
                Serial.println(" read).");
            #endif

            return count;
        }
        int8_t readWords(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint16_t *data, uint16_t timeout=I2CDEV_DEFAULT_READ_TIMEOUT) {
			#ifdef I2CDEV_SERIAL_DEBUG
				Serial.print("I2C (0x");
				Serial.print(devAddr, HEX);
				Serial.print(") reading ");
				Serial.print(length, DEC);
				Serial.print(" words from 0x");
				Serial.print(regAddr, HEX);
				Serial.print("...");
			#endif

			int8_t count = 0;
			uint32_t t1 = millis();

			// Arduino v1.0.1+, Wire library
			// Adds official support for repeated start condition, yay!

			// I2C/TWI subsystem uses internal buffer that breaks with large data requests
			// so if user requests more than BUFFER_LENGTH bytes, we have to do it in
			// smaller chunks instead of all at once
			for (uint8_t k = 0; k < length * 2; k += length * 2 < BUFFER_LENGTH ? length * 2 : BUFFER_LENGTH) {
				wire_.beginTransmission(devAddr);
				wire_.write(regAddr);
				wire_.endTransmission();
				wire_.beginTransmission(devAddr);
				wire_.requestFrom(devAddr, (uint8_t)(length * 2)); // length=words, this wants bytes

				bool msb = true; // starts with MSB, then LSB
				for (; wire_.available() && count < length && (timeout == 0 || millis() - t1 < timeout);) {
					if (msb) {
						// first byte is bits 15-8 (MSb=15)
						data[count] = wire_.read() << 8;
					} else {
						// second byte is bits 7-0 (LSb=0)
						data[count] |= wire_.read();
						#ifdef I2CDEV_SERIAL_DEBUG
							Serial.print(data[count], HEX);
							if (count + 1 < length) Serial.print(" ");
						#endif
						count++;
					}
					msb = !msb;
				}

				wire_.endTransmission();
			}

            if (timeout > 0 && millis() - t1 >= timeout && count < length) count = -1; // timeout

            #ifdef I2CDEV_SERIAL_DEBUG
                Serial.print(". Done (");
                Serial.print(count, DEC);
                Serial.println(" read).");
            #endif

            return count;
        }

        bool writeBit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t data) {
            uint8_t b;
            readByte(devAddr, regAddr, &b);
            b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
            return writeByte(devAddr, regAddr, b);
        }
        bool writeBitW(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint16_t data) {
            uint16_t w;
            readWord(devAddr, regAddr, &w);
            w = (data != 0) ? (w | (1 << bitNum)) : (w & ~(1 << bitNum));
            return writeWord(devAddr, regAddr, w);
        }
        bool writeBits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data) {
            //      010 value to write
            // 76543210 bit numbers
            //    xxx   args: bitStart=4, length=3
            // 00011100 mask byte
            // 10101111 original value (sample)
            // 10100011 original & ~mask
            // 10101011 masked | value
            uint8_t b;
            if (readByte(devAddr, regAddr, &b) != 0) {
                uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
                data <<= (bitStart - length + 1); // shift data into correct position
                data &= mask; // zero all non-important bits in data
                b &= ~(mask); // zero all important bits in existing byte
                b |= data; // combine data with existing byte
                return writeByte(devAddr, regAddr, b);
            } else {
                return false;
            }
        }
        bool writeBitsW(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint16_t data) {
            //              010 value to write
            // fedcba9876543210 bit numbers
            //    xxx           args: bitStart=12, length=3
            // 0001110000000000 mask word
            // 1010111110010110 original value (sample)
            // 1010001110010110 original & ~mask
            // 1010101110010110 masked | value
            uint16_t w;
            if (readWord(devAddr, regAddr, &w) != 0) {
                uint16_t mask = ((1 << length) - 1) << (bitStart - length + 1);
                data <<= (bitStart - length + 1); // shift data into correct position
                data &= mask; // zero all non-important bits in data
                w &= ~(mask); // zero all important bits in existing word
                w |= data; // combine data with existing word
                return writeWord(devAddr, regAddr, w);
            } else {
                return false;
            }
        }
        bool writeByte(uint8_t devAddr, uint8_t regAddr, uint8_t data) {
            return writeBytes(devAddr, regAddr, 1, &data);
        }
        bool writeWord(uint8_t devAddr, uint8_t regAddr, uint16_t data) {
            return writeWords(devAddr, regAddr, 1, &data);
        }
        bool writeBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data) {
            #ifdef I2CDEV_SERIAL_DEBUG
                Serial.print("I2C (0x");
                Serial.print(devAddr, HEX);
                Serial.print(") writing ");
                Serial.print(length, DEC);
                Serial.print(" bytes to 0x");
                Serial.print(regAddr, HEX);
                Serial.print("...");
            #endif
            uint8_t status = 0;
            wire_.beginTransmission(devAddr);
            wire_.write((uint8_t) regAddr); // send address
            for (uint8_t i = 0; i < length; i++) {
                #ifdef I2CDEV_SERIAL_DEBUG
                    Serial.print(data[i], HEX);
                    if (i + 1 < length) Serial.print(" ");
                #endif
                wire_.write((uint8_t) data[i]);
            }
            status = wire_.endTransmission();
            #ifdef I2CDEV_SERIAL_DEBUG
                Serial.println(". Done.");
            #endif
            return status == 0;
        }
        bool writeWords(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint16_t *data) {
            #ifdef I2CDEV_SERIAL_DEBUG
                Serial.print("I2C (0x");
                Serial.print(devAddr, HEX);
                Serial.print(") writing ");
                Serial.print(length, DEC);
                Serial.print(" words to 0x");
                Serial.print(regAddr, HEX);
                Serial.print("...");
            #endif
            uint8_t status = 0;
            wire_.beginTransmission(devAddr);
            wire_.write(regAddr); // send address
            for (uint8_t i = 0; i < length * 2; i++) {
                #ifdef I2CDEV_SERIAL_DEBUG
                    Serial.print(data[i], HEX);
                    if (i + 1 < length) Serial.print(" ");
                #endif
                wire_.write((uint8_t)(data[i] >> 8));    // send MSB
                wire_.write((uint8_t)data[i++]);         // send LSB
            }
            status = wire_.endTransmission();
            #ifdef I2CDEV_SERIAL_DEBUG
                Serial.println(". Done.");
            #endif
            return status == 0;
        }

    private:
        I2c& wire_;
};

#endif /* _I2CDEV_H_ */
