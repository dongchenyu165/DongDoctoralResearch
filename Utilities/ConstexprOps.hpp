#ifndef C4F7B28C_AA22_4303_8562_1AD4AAB80FF7
#define C4F7B28C_AA22_4303_8562_1AD4AAB80FF7

#if false
namespace __Internal__
{
	// 3.1 Define the to_chars struct
	// 		with a static member value[] of type const char[].
	// 		Specialized to:
	// 		struct to_chars<3, 5, 2, 7> { static const char value[]; };
    template<unsigned... Digits>
    struct to_chars { static const char value[]; };

	// 3.2 Assign the value of the static member value[]
	// 		by concatenating the character representations of the Digits.
	//		final [0] is the '\0'.
	// 		Assign to variable [ to_chars<3, 5, 2, 7>::value[] ]
	// 		to_chars<3, 5, 2, 7>::value[] = {('0' + 3), ('0' + 5), ('0' + 2), ('0' + 7), 0};
    template<unsigned... Digits>
    constexpr char to_chars<Digits...>::value[] = {('0' + Digits)..., 0};

	// 1.1 Template input {Expansion<3527>}
	// 1.2 Recursion specialize until the [Remain] is 0. {Expansion<0, 3, 5, 2, 7>}
	// 			Expansion<3527>
	// 			Expansion<352, 7>
	// 			Expansion<35, 2, 7>
	// 			Expansion<3, 5, 2, 7>
	// 			Expansion<0, 3, 5, 2, 7>
    template<unsigned Remain, unsigned... Digits>
    struct Expansion : Expansion<Remain / 10, Remain % 10, Digits...> {};

	// 2. Recursion end condition
	// 2.2 Specialize the base struct to_chars<Digits...>. {to_chars<3, 5, 2, 7>}
    template<unsigned... Digits>
    struct Expansion<0, Digits...> : to_chars<Digits...> {};
}

template<unsigned InNum>
struct ExprIntToString : __Internal__::Expansion<InNum> {};
#endif



#include <utility>
#include <type_traits>
#include <limits>

template<int N>
struct c_string
{
    int length;
    char str[N+1];

    constexpr explicit c_string(int p_length)
        : length(p_length), str{}
    {}
};

template<int M>
constexpr auto make_c_string(char const (&str)[M])
{
    c_string<M-1> ret{M-1};
    for(int i = 0; i < M; ++i)
    {
        ret.str[i] = str[i];
    }
    return ret;
}

template<int N, int M>
constexpr auto join(c_string<N> const& x, c_string<M> const& y)
{
    c_string<N+M> ret{x.length + y.length};

    for(int i = 0; i < x.length; ++i)
    {
        ret.str[i] = x.str[i];
    }
    for(int i = 0; i < y.length; ++i)
    {
        ret.str[i+x.length] = y.str[i];
    }

    ret.str[N+M] = '\0';

    return ret;
}

template<int N, int M>
constexpr auto operator+(c_string<N> const& x, c_string<M> const& y)
{
    return join(x, y);
}


template<class T>
constexpr void c_swap(T& x, T& y)
{
    T tmp( std::move(x) );
    x = std::move(y);
    y = std::move(tmp);
}
template<class T>
constexpr auto c_abs(T x)
{
    return x < T{0} ? -x : x;
}

template<class T>
constexpr auto ntoa(T n)
{
    c_string< std::numeric_limits<T>::digits10 + 1 > ret{0};
    int pos = 0;

    T cn = n;
    do
    {
        ret.str[pos] = '0' + c_abs(cn % 10);
        ++pos;
        cn /= 10;
    }while(cn != T{0});

    if(n < T{0})
    {
        ret.str[pos] = '-';
        ++pos;
    }

    ret.str[pos] = '\0';
    ret.length = pos;

    reverse(ret.str, ret.str+ret.length);
    return ret;
}

// not supported by the libstdc++ at coliru
//template<class T, class = std::enable_if_t< std::is_arithmetic<T>{} >>
template<class T, class = typename std::enable_if<std::is_arithmetic<T>{}>::type>
constexpr auto to_c_string(T p)
{
    return ntoa(p);
}
template<int N>
constexpr auto to_c_string(char const (&str)[N])
{
    return make_c_string(str);
}

template<class T, class U, class... TT>
constexpr auto to_c_string(T&& p0, U&& p1, TT&&... params)
{
    return   to_c_string(std::forward<T>(p0))
           + to_c_string(std::forward<U>(p1), std::forward<TT>(params)...);
}

#endif /* C4F7B28C_AA22_4303_8562_1AD4AAB80FF7 */
