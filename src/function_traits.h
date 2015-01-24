#ifndef _FUNCTION_TRAITS_H
#define _FUNCTION_TRAITS_H

#include <tuple>
#include <type_traits>

namespace scc {
	/* usage: decltype(return_args(&Class::memberfunction)) */
	template<typename Base, typename Result, typename... Args>
	std::tuple<Args...> return_args(Result (Base::*func)(Args...));

	/* usage: decltype(result_of(&Class::memberfunction)) */
	template<typename Base, typename Result, typename... Args>
	Result result_of(Result (Base::*func)(Args...));

	/* usage: arg_remove_reference<0, decltype(return_args(&Class::memberfunction))> */
	template <size_t i, typename Tuple>
	using decay_tuple_element = typename std::decay<typename std::tuple_element<i, Tuple>::type>::type;


	struct _is_static_castable_impl
	{
		template<typename _From, typename _To, typename = decltype(static_cast<_To>(std::declval<_From>()))>
		static std::true_type __test(int);

		template<typename, typename>
		static std::false_type __test(...);
	};

	template<typename _From, typename _To>
	struct is_static_castable
	: public _is_static_castable_impl
	{
		typedef decltype(__test<_From, _To>(0)) type;
	};

	template<typename _From, typename _To>
	using is_static_castable_t = typename is_static_castable<_From, _To>::type;
}

#endif
