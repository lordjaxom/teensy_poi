#ifndef LIBCPP_FUNCTION_HPP
#define LIBCPP_FUNCTION_HPP

#include <utility>

namespace libcpp {

    namespace detail {

        template< typename... Args >
        struct variadic_typedef
        {
        };

        template< typename Func >
        struct mem_fn_traits;

        template< typename T, typename Result, typename... Args >
        struct mem_fn_traits< Result ( T::* )( Args... ) >
        {
            using result_type = Result;
            using class_type = T;
            using args_type = variadic_typedef< Args... >;
            using signature = Result ( Args... );
        };

        template< typename T, typename Result, typename... Args >
        struct mem_fn_binder;

        template< typename T, typename Result, typename... Args >
        struct mem_fn_binder< T, Result, variadic_typedef< Args... > >
        {
            template< Result ( T::*MemFn )( Args... ) >
            struct binder
            {
                static Result invoke( void* instance, Args&&... args )
                {
                    ( static_cast< T* >( instance )->*MemFn )( std::forward< Args >( args )... );
                }

                binder( T& instance )
                    : instance_( &instance )
                {
                }

                T* instance_;
            };
        };

    } // namespace detail

    template< typename T >
    class function;

    template< typename Result, typename... Args >
    class function< Result ( Args... ) >
    {
    public:
        typedef Result ( *invoke_type )( void*, Args&&... );

        function()
			: invoke_()
		{
		}

        template< typename Binder >
        explicit function( Binder const& binder )
            : invoke_( &Binder::invoke )
            , instance_( binder.instance_ )
        {
        }

        operator bool() const
        {
        	return invoke_ != nullptr;
        }

        Result operator()( Args&&... args ) const
        {
        	return invoke_( instance_, std::forward< Args >( args )... );
        }

    private:
        invoke_type invoke_;
        void* instance_;
    };

#define libcpp__mem_fn( func_, inst_ ) \
    ::libcpp::function< \
        typename ::libcpp::detail::mem_fn_traits< decltype( func_ ) >::signature \
    >( typename ::libcpp::detail::mem_fn_binder< \
        typename ::libcpp::detail::mem_fn_traits< decltype( func_ ) >::class_type, \
        typename ::libcpp::detail::mem_fn_traits< decltype( func_ ) >::result_type, \
        typename ::libcpp::detail::mem_fn_traits< decltype( func_ ) >::args_type \
    >::template binder< func_ >( inst_ ) )

} // namespace libcpp

#endif // LIBCPP_FUNCTION_HPP
