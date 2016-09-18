#ifndef TEENSY_POI_PROTOCOL_HPP
#define TEENSY_POI_PROTOCOL_HPP

#include <stddef.h>
#include <stdint.h>

#include "config.hpp"

static uint8_t constexpr magicPacket[]     = { 0xaf, 0xb9, 0x9e };
static size_t  constexpr magicPacketLength = sizeof( magicPacket );

static uint8_t constexpr commandHello      = 0x01;
static uint8_t constexpr commandRead       = 0x02;
static uint8_t constexpr commandWrite      = 0x03;

static uint8_t constexpr responseOk        = 0x01;
static uint8_t constexpr responseAwaitData = 0x02;
static uint8_t constexpr responseError     = 0x03;

static size_t  constexpr flashSize         = 2 * 1024 * 1024;
static size_t  constexpr flashBlockSize    = 4096;
static size_t  constexpr flashBlockCount   = flashSize / flashBlockSize;
static uint8_t constexpr flashBlockVersion = 0x03;

enum class FlashFileType : uint8_t
{
	IMAGE  = 0x01,
	EOD    = 0xff
};

struct FlashFileHeader
{
	uint8_t        version;
	FlashFileType  fileType;
	uint32_t       fileSize; // excluding headers
} __attribute__(( packed ));

struct FlashImageFileHeader
{
	FlashFileHeader fileHeader;
	uint16_t        linesPerRound;
} __attribute__(( packed ));

static size_t constexpr flashHeaderMaxSize = sizeof( FlashImageFileHeader );

#endif // TEENSY_POI_PROTOCOL_HPP
