#include <fxp/fxp.hpp>
#include <type_traits>

using namespace fxp::literals;
static_assert(std::is_same_v<decltype(1.2_r + 4.00002345_fine), fxp::fine>);
static_assert(fxp::real(123) == 123_r);
static_assert(fxp::fine(1.5_r) == 1.5_fine);

// Floating point is permitted in the same translation unit.
double unrelated_floating_point(double value) { return value + 1.23; }
