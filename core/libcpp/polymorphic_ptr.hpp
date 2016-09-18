#ifndef LIBCPP_POLYMORPHIC_PTR_HPP
#define LIBCPP_POLYMORPHIC_PTR_HPP

#include <utility>

#include <libcpp/type_traits.hpp>

namespace libcpp {

	template< typename T >
	struct polymorphic_type {};


	struct polymorphic_element
	{
		polymorphic_element( size_t index ) : index( index ) {}
		size_t index;
	};


	namespace detail {

		template< typename U, typename Base, typename... T >
		struct polymorphic_construct_by_type;

		template< typename U, typename Base, typename... T >
		struct polymorphic_construct_by_type< U, Base, U, T... >
		{
			template< typename... Args >
			static Base* construct( void* storage, Args&&... args )
			{
				return new ( storage ) U( std::forward< Args >( args )... );
			}
		};

		template< typename U, typename Base, typename T0, typename... T >
		struct polymorphic_construct_by_type< U, Base, T0, T... >
		{
			template< typename... Args >
			static Base* construct( void* storage, Args&&... args )
			{
				return polymorphic_construct_by_type< U, Base, T... >::construct(
						storage, std::forward< Args >( args )... );
			}
		};


		template< typename Base, typename... T >
		struct polymorphic_construct_by_index
		{
			template< size_t I = 0, typename... Args >
			static Base* construct( size_t index, void* storage, Args&&... args )
			{
				return nullptr;
			}
		};

		template< typename Base, typename T0, typename... T >
		struct polymorphic_construct_by_index< Base, T0, T... >
		{
			template< size_t I = 0, typename... Args >
			static Base* construct( size_t index, void* storage, Args&&... args )
			{
				return I == index
						? new ( storage ) T0( std::forward< Args >( args )... )
						: polymorphic_construct_by_index< Base, T... >::template construct< I + 1 >(
							index, storage, std::forward< Args >( args )... );
			}
		};


		template< typename U, typename Base, typename... T >
		struct polymorphic_construct;

		template< typename U, typename Base, typename... T >
		struct polymorphic_construct< polymorphic_type< U >, Base, T... >
		{
			template< typename... Args >
			static Base* construct( polymorphic_type< U >, void* storage, Args&&... args )
			{
				return polymorphic_construct_by_type< U, Base, T... >::construct(
						storage, std::forward< Args >( args )... );
			}
		};

		template< typename Base, typename... T >
		struct polymorphic_construct< polymorphic_element, Base, T... >
		{
			template< typename... Args >
			static Base* construct( polymorphic_element const& element, void* storage, Args&&... args )
			{
				return polymorphic_construct_by_index< Base, T... >::construct(
						element.index, storage, std::forward< Args >( args )... );
			}
		};

	} // namespace detail


	template< typename Base, typename... T >
	class polymorphic_ptr
	{
		using storage_type = aligned_union< 0, T... >;

	public:
		polymorphic_ptr()
			: pointer_()
		{
		}

		template< typename U, typename... Args >
		explicit polymorphic_ptr( U const& what, Args&&... args )
			: pointer_( detail::polymorphic_construct< U, Base, T... >::construct(
					what, &storage_, std::forward< Args >( args )... ) )
		{
		}

		polymorphic_ptr( polymorphic_ptr const& ) = delete;

		~polymorphic_ptr()
		{
			destruct();
		}

		operator bool() const { return pointer_ != nullptr; }

		Base& operator*() { return *pointer_; }
		Base const& operator*() const { return *pointer_; }
		Base* operator->() { return pointer_; }
		Base const* operator->() const { return pointer_; }

		template< typename U, typename... Args >
		void reset( U const& what, Args&&... args )
		{
			destruct();
			pointer_ = detail::polymorphic_construct< U, Base, T... >::construct(
					what, &storage_, std::forward< Args >( args )... );
		}

		void reset( std::nullptr_t )
		{
			destruct();
			pointer_ = nullptr;
		}

	private:
		void destruct()
		{
			if ( pointer_ != nullptr ) {
				pointer_->~Base();
			}
		}

		storage_type storage_;
		Base* pointer_;
	};

} // namespace libcpp

#endif // LIBCPP_POLYMORPHIC_PTR_HPP
