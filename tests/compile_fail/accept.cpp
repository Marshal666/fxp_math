#include <fxp/fxp.hpp>
#include <type_traits>

using namespace fxp::literals;
static_assert(std::is_same_v<decltype(1.2_r + 4.00002345_fine), fxp::fine>);
static_assert(fxp::real(123) == 123_r);
static_assert(fxp::fine(1.5_r) == 1.5_fine);

// Exercise both constrained constructor forms under the consumer's compiler
// settings, including mixed formats, rejected coordinates, and wrong arities.
template <class T, class U> constexpr bool vector_constructors() {
    const fxp::vec2<T> pair{1, 2};
    const fxp::vec3<T> components{1, T(2), U(3)};
    const fxp::vec4<T> from_pair{pair, U(3), 4};
    const fxp::vec4<T> from_triple{fxp::vec3<U>{1, 2, 3}, 4};
    const fxp::vec3<T> converted{fxp::vec3<U>{1, 2, 3}};
    const fxp::vec3<T> copied{components};
    const fxp::vec3<T> zero{};

    static_assert(!std::is_constructible_v<fxp::vec3<T>, float, int, int>);
    static_assert(!std::is_constructible_v<fxp::vec3<T>, int, double, int>);
    static_assert(!std::is_constructible_v<fxp::vec3<T>, int, int, float>);
    static_assert(!std::is_constructible_v<fxp::vec4<T>, fxp::vec2<U>, float, int>);
    static_assert(!std::is_constructible_v<fxp::vec4<T>, fxp::vec2<U>, int, double>);
    static_assert(!std::is_constructible_v<fxp::vec4<T>, fxp::vec3<U>, double>);
    static_assert(!std::is_constructible_v<fxp::vec3<T>, int, int>);
    static_assert(!std::is_constructible_v<fxp::vec3<T>, int, int, int, int>);
    static_assert(!std::is_constructible_v<fxp::vec4<T>, fxp::vec2<U>, int>);
    static_assert(!std::is_constructible_v<fxp::vec4<T>, fxp::vec3<U>, int, int>);
    static_assert(!std::is_constructible_v<fxp::vec3<T>, fxp::vec4<U>, int>);

    return components.x == 1 && components.y == 2 && components.z == 3 &&
           from_pair == fxp::vec4<T>{1, 2, 3, 4} && from_triple == from_pair &&
           converted == components && copied == components &&
           zero.x == 0 && zero.y == 0 && zero.z == 0;
}
static_assert(vector_constructors<fxp::real, fxp::real>());
static_assert(vector_constructors<fxp::fine, fxp::fine>());
static_assert(vector_constructors<fxp::real, fxp::fine>());
static_assert(vector_constructors<fxp::fine, fxp::real>());

// Floating point is permitted in the same translation unit.
double unrelated_floating_point(double value) { return value + 1.23; }
