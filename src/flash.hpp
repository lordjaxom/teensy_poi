#ifndef TEENSY_POI_FLASH_HPP
#define TEENSY_POI_FLASH_HPP

#include <limits>

#include <SPI.h>
#include <SPIFlash.h>

#include "protocol.hpp"

namespace tp {

	namespace detail {

		template< typename T >
		struct HeaderTraits;

		template<>
		struct HeaderTraits< FlashImageFileHeader >
		{
			static FlashFileType constexpr fileType = FlashFileType::IMAGE;
		};


		template< FlashFileType Type >
		struct FileTraits;

		template<>
		struct FileTraits< FlashFileType::IMAGE >
		{
            using HeaderType = FlashImageFileHeader;
		};


		template< FlashFileType Type >
		struct FileContent;

		template<>
		struct FileContent< FlashFileType::IMAGE >
		{
		    FileContent()
		    {
		    }

            FileContent( FlashImageFileHeader const* header )
                : linesPerRound_( header->linesPerRound )
            {
            }

            uint16_t linesPerRound() const { return linesPerRound_; }

        private:
            uint16_t linesPerRound_;
		};


		inline uint32_t nextBlockAddr( uint32_t startAddr, uint32_t fileSize )
		{
		    uint32_t endAddr = startAddr + fileSize;
		    uint32_t lastBlockSize = endAddr % flashBlockSize;
		    if ( lastBlockSize > 0 ) {
                endAddr += flashBlockSize - lastBlockSize;
		    }
		    return endAddr;
		}

	} // namespace detail

	template< typename T >
	T const* headerCast( FlashFileHeader const* header )
	{
		return header->fileType == detail::HeaderTraits< T >::fileType
			? reinterpret_cast< T const* >( header )
			: nullptr;
	}

	template< typename Mosi, typename Miso, typename Sck, typename Csel >
	class Flash;

	template< FlashFileType Type >
	class FlashFile
	{
	    using FileTraits = detail::FileTraits< Type >;
	    using FileContent = detail::FileContent< Type >;

		static uint32_t constexpr nullAddr = std::numeric_limits< uint32_t >::max();

    public:
	    using HeaderType = typename FileTraits::HeaderType;
	    static size_t constexpr headerSize = sizeof( typename FileTraits::HeaderType );

        FlashFile()
            : startAddr_( nullAddr )
        {
        }

        FlashFile( uint32_t startAddr, typename FileTraits::HeaderType const* header )
            : startAddr_( startAddr )
            , fileSize_( header->fileHeader.fileSize )
            , currentAddr_( startAddr_ + headerSize )
            , content_( header )
        {
        }

        operator bool() const { return startAddr_ != nullAddr; }

        uint32_t currentAddr() const { return currentAddr_; }
        FileContent const& content() const { return content_; }

        uint32_t nextBlockAddr() const
        {
        	if ( startAddr_ == nullAddr ) {
				return 0;
        	}
            uint32_t endAddr = startAddr_ + headerSize + fileSize_;
		    uint32_t lastBlockSize = endAddr % flashBlockSize;
		    if ( lastBlockSize > 0 ) {
                endAddr += flashBlockSize - lastBlockSize;
		    }
		    return endAddr;
        }

        void advance( uint16_t length )
        {
            currentAddr_ += length;
            if ( currentAddr_ >= startAddr_ + headerSize + fileSize_ ) {
                currentAddr_ = startAddr_ + headerSize;
            }
        }

    private:
        uint32_t startAddr_;
        uint32_t fileSize_;
        uint32_t currentAddr_;
        FileContent content_;
	};


	template< typename Mosi, typename Miso, typename Sck, typename Csel >
	class Flash
	{
		using SpiType = SPI< Mosi, Miso, Sck >;
		using FlashType = SPIFlash< SpiType, Csel >;

		using FlashHeaderBuffer = uint8_t[ flashHeaderMaxSize ];

	public:
        using Image = FlashFile< FlashFileType::IMAGE >;

		Flash()
			: flash_( spi_ )
			, ok_( true )
		{
		}

		Flash( Flash const& ) = delete;

		operator bool() const
		{
			return ok_;
		}

		template< typename File >
		File next( File const& file )
		{
		    return next< File >( file.nextBlockAddr(), false );
		}

		template< typename File >
		void read( File& file, uint8_t* buffer, uint16_t size )
		{
			flash_.readBytes( file.currentAddr(), buffer, size );
			file.advance( size );
		}

		void readBytes( uint32_t address, uint8_t* buffer, uint16_t size )
		{
		    flash_.readBytes( address, buffer, size );
		}

		void writeBytes( uint32_t address, uint8_t const* buffer, uint16_t size )
		{
		    flash_.writeBytes( address, buffer, size );
		}

		void eraseBlock( uint32_t address )
		{
		    flash_.blockErase4K( address );
		}

	private:
	    template< typename File >
		File next( uint32_t addr, bool wrap )
		{
		    if ( addr >= flashSize ) {
                return wrap
                        ? next< File >( 0, false )
                        : File();
		    }

			FlashHeaderBuffer buffer;
			FlashFileHeader const* header = readHeader( addr, buffer );
			if ( header == nullptr ) {
                return File();
			}
			if ( header->fileType == FlashFileType::EOD ) {
			    return wrap
                    ? next< File >( 0, false )
                    : File();
			}

			typename File::HeaderType const* fileHeader = headerCast< typename File::HeaderType >( header );
			if ( fileHeader != nullptr ) {
				return File( addr, fileHeader );
			}
			// TODO look behind next file (if there's more than Images)

			return File();
		}

		FlashFileHeader const* readHeader( uint32_t addr, FlashHeaderBuffer& buffer )
		{
			flash_.readBytes( addr, &buffer[ 0 ], sizeof( buffer ) );
			FlashFileHeader* fileHeader = reinterpret_cast< FlashFileHeader* >( &buffer[ 0 ] );
			if ( fileHeader->version != flashBlockVersion ) {
				ok_ = false;
				return nullptr;
			}
			return fileHeader;
		}

		SpiType spi_;
		FlashType flash_;
		bool ok_;
	};

} // namespace tp

#endif // TEENSY_POI_FLASH_HPP
