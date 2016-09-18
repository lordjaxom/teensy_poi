#ifndef LIBCPP_TYPE_TRAITS_HPP
#define LIBCPP_TYPE_TRAITS_HPP

#include <type_traits>

#include <libcpp/numeric.hpp>

namespace libcpp {

	template< size_t Len, typename... T >
	using aligned_union = typename std::aligned_storage< max( Len, sizeof( T )... ) >::type;

} // namespace libcpp

#endif // LIBCPP_TYPE_TRAITS_HPP
