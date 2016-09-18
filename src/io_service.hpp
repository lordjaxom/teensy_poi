#ifndef TLIBCPP_IO_SERVICE_HPP
#define TLIBCPP_IO_SERVICE_HPP

#include <new>
#include <type_traits>
#include <tuple>
#include <utility>

#include <libcpp/function.hpp>
#include <libcpp/variant.hpp>

namespace tp {

    namespace IoServiceDetail {

    	template< typename... Variants >
    	struct ConcatVariants;

    	template< typename... Handlers0, typename... Handlers1 >
    	struct ConcatVariants< libcpp::variant< Handlers0... >, libcpp::variant< Handlers1... > >
    	{
    		using Type = libcpp::variant< Handlers0..., Handlers1... >;
    	};


 		template< typename IoService, template< typename > class... Channels >
 		struct ChannelList
 		{
 			using HandlersType = libcpp::variant<>;
 		};

 		template< typename IoService, template< typename > class Channel0, template< typename > class... Channels >
 		struct ChannelList< IoService, Channel0, Channels... >
 		{
 			using HandlersType = typename ConcatVariants<
					typename Channel0< IoService >::HandlersType,
					typename ChannelList< IoService, Channels... >::HandlersType >::Type;
		};


	    struct HandlerInvokeVisitor
        {
            template< typename T >
            bool operator()( T& handler, uint32_t timeout )
            {
                return handler( timeout );
            }
        };

    } // namespace IoServiceDetail


    template< typename... Handlers >
    class IoChannel
    {
    public:
        using HandlersType = libcpp::variant< Handlers... >;
    };


	template< uint32_t TimeoutUs, template< typename > class... Channels >
    class IoService
    {
    	static constexpr size_t queueLength = 2;

    	using ChannelList = IoServiceDetail::ChannelList< IoService, Channels... >;
    	using HandlersType = typename ChannelList::HandlersType;

    public:
    	explicit IoService( Stopwatch const& stopwatch )
			: stopwatch_( stopwatch )
			, queueIndex_()
			, queueUsed_()
		{
		}

		IoService( IoService const& ) = delete;

		template< typename Handler >
		void invoke( Handler&& handler )
		{
			if ( queueUsed_ == queueLength ) {
				return;
			}
			auto&& handlerSlot = handlerQueue_[ ( queueIndex_+ queueUsed_ ) % queueLength ];
			handlerSlot = std::forward< Handler >( handler );
			++queueUsed_;
		}

		void advance()
		{
			if ( queueUsed_ == 0 ) {
				return;
			}
			uint32_t remaining;
			while ( queueUsed_ > 0 && ( remaining = stopwatch_.remaining( TimeoutUs ) ) > 0 ) {
				auto&& handlerSlot = handlerQueue_[ queueIndex_ ];
				if ( handlerSlot.visit( IoServiceDetail::HandlerInvokeVisitor(), remaining ) ) {
					handlerSlot.reset();
					queueIndex_ = ( queueIndex_ + 1 ) % queueLength;
					--queueUsed_;
				}
			}
		}

    private:
    	Stopwatch const& stopwatch_;
		HandlersType handlerQueue_[ queueLength ];
		size_t queueIndex_;
		size_t queueUsed_;
    };

} // namespace tp

#endif // TLIBCPP_IO_SERVICE_HPP
