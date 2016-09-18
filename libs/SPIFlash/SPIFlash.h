/*
 * Copyright (c) 2013 by Felix Rusu <felix@lowpowerlab.com>
 * SPI Flash memory library for arduino/moteino.
 * This works with 256byte/page SPI flash memory
 * For instance a 4MBit (512Kbyte) flash chip will have 2048 pages: 256*2048 = 524288 bytes (512Kbytes)
 * Minimal modifications should allow chips that have different page size but modifications
 * DEPENDS ON: Arduino SPI library
 *
 * This file is free software; you can redistribute it and/or modify
 * it under the terms of either the GNU General Public License version 2
 * or the GNU Lesser General Public License version 2.1, both as
 * published by the Free Software Foundation.
 */

#ifndef _SPIFLASH_H_
#define _SPIFLASH_H_

#define USE_AAIWORDPROGRAM

#include <SPI.h>

/// IMPORTANT: NAND FLASH memory requires erase before write, because
///            it can only transition from 1s to 0s and only the erase command can reset all 0s to 1s
/// See http://en.wikipedia.org/wiki/Flash_memory
/// The smallest range that can be erased is a sector (4K, 32K, 64K); there is also a chip erase command

/// Standard SPI flash commands
/// Assuming the WP pin is pulled up (to disable hardware write protection)
/// To use any write commands the WEL bit in the status register must be set to 1.
/// This is accomplished by sending a 0x06 command before any such write/erase command.
/// The WEL bit in the status register resets to the logical “0” state after a
/// device power-up or reset. In addition, the WEL bit will be reset to the logical “0” state automatically under the following conditions:
/// • Write Disable operation completes successfully
/// • Write Status Register operation completes successfully or aborts
/// • Protect Sector operation completes successfully or aborts
/// • Unprotect Sector operation completes successfully or aborts
/// • Byte/Page Program operation completes successfully or aborts
/// • Sequential Program Mode reaches highest unprotected memory location
/// • Sequential Program Mode reaches the end of the memory array
/// • Sequential Program Mode aborts
/// • Block Erase operation completes successfully or aborts
/// • Chip Erase operation completes successfully or aborts
/// • Hold condition aborts
#define SPIFLASH_WRITEENABLE      0x06        // write enable
#define SPIFLASH_WRITEDISABLE     0x04        // write disable

#define SPIFLASH_BLOCKERASE_4K    0x20        // erase one 4K block of flash memory
#define SPIFLASH_BLOCKERASE_32K   0x52        // erase one 32K block of flash memory
#define SPIFLASH_BLOCKERASE_64K   0xD8        // erase one 64K block of flash memory
#define SPIFLASH_CHIPERASE        0x60        // chip erase (may take several seconds depending on size)
                                              // but no actual need to wait for completion (instead need to check the status register BUSY bit)
#define SPIFLASH_STATUSREAD       0x05        // read status register
#define SPIFLASH_STATUSWRITE      0x01        // write status register
#define SPIFLASH_ARRAYREAD        0x0B        // read array (fast, need to add 1 dummy byte after 3 address bytes)
#define SPIFLASH_ARRAYREADLOWFREQ 0x03        // read array (low frequency)

#define SPIFLASH_BYTEPROGRAM      0x02        // write 1 byte
#define SPIFLASH_AAIWORDPROGRAM   0xAD        // write continuously with auto-address-increment
#define SPIFLASH_IDREAD           0x9F        // read JEDEC manufacturer and device ID (2 bytes, specific bytes for each manufacturer and device)
                                              // Example for Atmel-Adesto 4Mbit AT25DF041A: 0x1F44 (page 27: http://www.adestotech.com/sites/default/files/datasheets/doc3668.pdf)
                                              // Example for Winbond 4Mbit W25X40CL: 0xEF30 (page 14: http://www.winbond.com/NR/rdonlyres/6E25084C-0BFE-4B25-903D-AE10221A0929/0/W25X40CL.pdf)
#define SPIFLASH_MACREAD          0x4B        // read unique ID number (MAC)

template< typename Spi, typename Csel >
class SPIFlash {
public:
	SPIFlash(Spi& spi): spi_(spi)
	{
		Csel::output();
		Csel::set();

		command<true>(SPIFLASH_STATUSWRITE); // Write Status Register
		spi_.transfer(0);                    // Global Unprotect
		unselect();
	}

	template<bool Write = false, bool HighSpeed = false>
	void command(uint8_t cmd)
	{
		if (Write) {
			command(SPIFLASH_WRITEENABLE); // Write Enable
			unselect();
		}
		while(busy()); //wait for any write/erase to complete
		select<HighSpeed>();
		spi_.transfer(cmd);
	}

	uint8_t readStatus()
	{
		select();
		spi_.transfer(SPIFLASH_STATUSREAD);
		uint8_t status = spi_.transfer(0);
		unselect();
		return status;
	}

	uint8_t readByte(uint32_t addr)
	{
		command(SPIFLASH_ARRAYREADLOWFREQ);
		spi_.transfer(addr >> 16);
		spi_.transfer(addr >> 8);
		spi_.transfer(addr);
		uint8_t result = spi_.transfer(0);
		unselect();
		return result;
	}

	void readBytes(uint32_t addr, uint8_t* buf, uint16_t len)
	{
		command<false, true>(SPIFLASH_ARRAYREAD);
		spi_.transfer(addr >> 16);
		spi_.transfer(addr >> 8);
		spi_.transfer(addr);
		spi_.transfer(0); //"dont care"
		while (len-- > 0)
			*buf++ = spi_.transfer(0);
		unselect();
	}

	void writeByte(uint32_t addr, uint8_t byt)
	{
		command<true>(SPIFLASH_BYTEPROGRAM);  // Byte Program
		spi_.transfer(addr >> 16);
		spi_.transfer(addr >> 8);
		spi_.transfer(addr);
		spi_.transfer(byt);
		unselect();
	}

	void writeBytes(uint32_t addr, const uint8_t* buf, uint16_t len)
	{
#ifdef USE_AAIWORDPROGRAM
		// AAI writes are always at even addrs, so send odd byte regularly
		if (addr & 1) {
			writeByte(addr++, *buf++);
			--len;
		}

		if (len >= 2) {
			uint16_t even = len & ~1; // even byte count

			command<true>(SPIFLASH_AAIWORDPROGRAM);
			spi_.transfer(addr >> 16);
			spi_.transfer(addr >> 8);
			spi_.transfer(addr);
			spi_.transfer(*buf++);
			spi_.transfer(*buf++);
			unselect();

			len -= even; // odd bytes remaining
			addr += even; // addr of odd bytes (current address not needed during write)
			even -= 2; // two bytes already written above

			while (even > 0) {
				command(SPIFLASH_AAIWORDPROGRAM);
				spi_.transfer(*buf++);
				spi_.transfer(*buf++);
				unselect();
				even -= 2;
			}
			command(SPIFLASH_WRITEDISABLE);
		}

		if (len > 0)
			writeByte(addr, *buf);
#else
		for (uint16_t i = 0; i < len; i++)
			writeByte(addr++, *buf++);
#endif // USE_AAIWORDPROGRAM
	}

	bool busy() { return readStatus() & 1; }

	void chipErase()
	{
		command<true>(SPIFLASH_CHIPERASE);
		unselect();
	}

	void blockErase4K(uint32_t addr)
	{
		command<true>(SPIFLASH_BLOCKERASE_4K); // Block Erase
		spi_.transfer(addr >> 16);
		spi_.transfer(addr >> 8);
		spi_.transfer(addr);
		unselect();
	}

	void blockErase32K(uint32_t addr)
	{
		command<true>(SPIFLASH_BLOCKERASE_32K); // Block Erase
		spi_.transfer(addr >> 16);
		spi_.transfer(addr >> 8);
		spi_.transfer(addr);
		unselect();
	}

private:
	template<bool HighSpeed=false>
	void select()
	{
		spi_.template beginTransaction<HighSpeed?80000000:25000000, MSBFIRST, SPI_MODE0>();
		Csel::clear();
	}

	void unselect()
	{
		Csel::set();
		spi_.endTransaction();
	}

	Spi& spi_;
};

#endif
