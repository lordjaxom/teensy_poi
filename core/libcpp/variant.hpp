#ifndef LIBCPP_VARIANT_HPP
#define LIBCPP_VARIANT_HPP

#include <limits>
#include <new>
#include <tuple>
#include <utility>
#ifndef LIBCPP_NO_EXCEPTIONS
#	include <typeinfo>
#endif

#include <libcpp/type_traits.hpp>

namespace libcpp {

	template< typename... T >
	class variant;

#ifdef LIBCPP_NO_EXCEPTIONS
    template< typename T >
    T variant_default_result()
    {
        return T();
    }
#endif

	namespace detail {

	    static constexpr size_t variant_not_found = std::numeric_limits< size_t >::max();


		struct variant_construct_by_index
		{
		    variant_construct_by_index( size_t index )
                : index_( index )
            {
            }

            size_t index_;
		};


		template< typename U >
		struct variant_construct_by_rvalue
		{
		};


		template< typename U >
		struct variant_construct_by_type
		{
		};


		template< typename U, typename... T >
		struct variant_constructor;

		template< typename U, typename... T >
		struct variant_constructor< variant_construct_by_rvalue< U >, U, T... >
		{
			template< size_t I = 0, typename Storage >
			static size_t construct( Storage& storage, variant_construct_by_rvalue< U >, U&& value )
			{
				new ( &storage ) U( std::forward< U >( value ) );
				return I;
			}
		};

		template< typename U, typename T0, typename... T >
		struct variant_constructor< variant_construct_by_rvalue< U >, T0, T... >
		{
			template< size_t I = 0, typename Storage >
			static size_t construct( Storage& storage, variant_construct_by_rvalue< U > const& what, U&& value )
			{
				return variant_constructor< variant_construct_by_rvalue< U >, T... >::template construct< I + 1 >(
						storage, what, std::forward< U >( value ) );
			}
		};

		template< typename U, typename... T >
		struct variant_constructor< variant_construct_by_type< U >, U, T... >
		{
		    template< size_t I = 0, typename Storage, typename... Args >
		    static size_t construct( Storage& storage, variant_construct_by_type< U >, Args&&... args )
		    {
		        new ( &storage ) U( std::forward< Args >( args )... );
		        return I;
		    }
		};

		template< typename U, typename T0, typename... T >
		struct variant_constructor< variant_construct_by_type< U >, T0, T... >
		{
		    template< size_t I = 0, typename Storage, typename... Args >
		    static size_t construct( Storage& storage, variant_construct_by_type< U > const& what, Args&&... args )
		    {
		        return variant_constructor< variant_construct_by_type< U >, T... >::template construct< I + 1 >(
                        storage, what, std::forward< Args >( args )... );
		    }
		};

		template<>
		struct variant_constructor< variant_construct_by_index >
		{
		    template< size_t I = 0, typename Storage, typename... Args >
			static size_t construct( Storage& storage, variant_construct_by_index const& value, Args&&... args )
			{
			    return variant_not_found;
			}
		};

		template< typename T0, typename... T >
		struct variant_constructor< variant_construct_by_index, T0, T... >
		{
		    template< size_t I = 0, typename Storage, typename... Args >
			static size_t construct( Storage& storage, variant_construct_by_index const& value, Args&&... args )
			{
			    if ( I == value.index_ ) {
                    new ( &storage ) T0( std::forward< Args >( args )... );
                    return I;
			    }
			    return variant_constructor< variant_construct_by_index, T... >::template construct< I + 1 >(
                        storage, value, std::forward< Args >( args )... );
			}
		};


		template< typename V, typename Visitor, typename Result, typename... Args >
		struct variant_visitation_impl
		{
			template< size_t I = 0, typename Storage >
			static Result visit( Storage& storage, size_t index, Visitor&& visitor, Args&&... args )
			{
#ifndef LIBCPP_NO_EXCEPTIONS
				throw std::bad_cast();
#else
				return variant_default_result< Result >();
#endif
			}
		};

		template< typename T0, typename... T, typename Visitor, typename Result, typename... Args >
		struct variant_visitation_impl< variant< T0, T... >, Visitor, Result, Args... >
		{
			template< size_t I = 0, typename Storage >
			static Result visit( Storage& storage, size_t index, Visitor&& visitor, Args&&... args )
			{
				return I == index
						? visitor( *reinterpret_cast< T0* >( &storage ), std::forward< Args >( args )... )
						: variant_visitation_impl< variant< T... >, Visitor, Result, Args... >::
							template visit< I + 1 >( storage, index, std::forward< Visitor >( visitor ),
								std::forward< Args >( args )... );
			}
		};


		template< typename V, typename Visitor, typename... Args >
		struct variant_visitation;

		template< typename T0, typename... T, typename Visitor, typename... Args >
		struct variant_visitation< variant< T0, T... >, Visitor, Args... >
		{
            using result_type = typename std::result_of< Visitor ( T0&, Args... ) >::type;

            template< typename Storage >
            static result_type visit( Storage& storage, size_t index, Visitor&& visitor, Args&&... args )
            {
            	return variant_visitation_impl< variant< T0, T... >, Visitor, result_type, Args... >::
						template visit< 0 >( storage, index, std::forward< Visitor >( visitor ),
							std::forward< Args >( args )... );
            }
		};


		struct variant_destruct_visitor
		{
			template< typename T >
			void operator()( T& value )
			{
				value.~T();
			}
		};


		struct variant_copy_visitor
		{
			template< typename T, typename... U >
			void operator()( T const& value, variant< U... >& dest ) const
			{
				dest = value;
			}
		};


		struct variant_move_visitor
		{
			template< typename T, typename... U >
			void operator()( T&& value, variant< U... >& dest ) const
			{
				dest = std::move< T >( value );
			}
		};


		template< typename T >
		struct variant_get_visitor
		{
			T* operator()( T& value ) const
			{
				return &value;
			}

			template< typename U >
			T* operator()( U const& value ) const
			{
				return nullptr;
			}
		};

	} // namespace detail

	template< typename... T >
	class variant
	{
		using storage_type = libcpp::aligned_union< 0, T... >;

		static constexpr size_t not_found = detail::variant_not_found;

	public:
		variant()
			: index_( not_found )
		{
		}

		variant( variant const& ) = delete;

		~variant()
		{
			destruct();
		}

		variant& operator=( variant const& ) = delete;
		variant& operator=( variant&& other ) = delete;

		template< typename U >
		variant& operator=( U&& value )
		{
		    destruct();
		    construct( detail::variant_construct_by_rvalue< U >(), std::forward< U >( value ) );
		    return *this;
		}

		operator bool() const { return index_ != not_found; }

		template< typename Visitor, typename... Args >
		typename detail::variant_visitation< variant, Visitor, Args... >::result_type
				visit( Visitor&& visitor, Args&&... args )
		{
			return detail::variant_visitation< variant, Visitor, Args... >::visit(
					storage_, index_, std::forward< Visitor >( visitor ), std::forward< Args >( args )... );
		}

		void reset()
		{
			destruct();
			index_ = not_found;
		}

		template< typename U, typename... Args >
		void emplace( Args&&... args )
		{
		    destruct();
		    construct( detail::variant_construct_by_type< U >(), std::forward< Args >( args )... );
		}

		template< typename... Args >
		void emplace( size_t index, Args&&... args )
		{
		    destruct();
		    construct( detail::variant_construct_by_index( index ), std::forward< Args >( args )... );
		}

		size_t index() const
		{
			return index_;
		}

	private:
		template< typename C, typename... Args >
		void construct( C const& what, Args&&... args )
		{
			index_ = detail::variant_constructor< C, T... >::
                    construct( storage_, what, std::forward< Args >( args )... );
		}

		void destruct()
		{
			if ( index_ != not_found ) {
				visit( detail::variant_destruct_visitor() );
			}
		}

		storage_type storage_;
		size_t index_;
	};

} // namespace libcpp

#endif // LIBCPP_VARIANT_HPP
