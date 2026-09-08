#include <fxp/fxp.hpp>
#include <gtest/gtest.h>
using namespace fxp::literals;
using fxp::real;
using fxp::fine;
template<class T> class MathTest : public ::testing::Test {};
using MathFormats = ::testing::Types<real, fine>;
TYPED_TEST_SUITE(MathTest, MathFormats, );
TYPED_TEST(MathTest, RemaindersAndRounding) {
    using T = TypeParam;
    EXPECT_EQ(fxp::fmod(T(-7), T(2)), T(-1));
    EXPECT_EQ(fxp::remainder(T(7), T(2)), T(-1));
    EXPECT_EQ(fxp::remainder(T(5), T(2)), T(1));
    EXPECT_EQ(fxp::remainder(T(-7), T(2)), T(1));
    int quotient = 99;
    EXPECT_EQ(fxp::remquo(T(7), T(-2), &quotient), T(-1)); EXPECT_EQ(quotient, -4);
    EXPECT_EQ(fxp::remquo(T(1025), T(2), &quotient), T(1)); EXPECT_EQ(quotient, 0);
    EXPECT_EQ(fxp::remainder(T::min(), T::from_raw(-1)), T{});
    const auto x = T::from_string("-2.5");
    EXPECT_EQ(fxp::floor(x), T(-3)); EXPECT_EQ(fxp::ceil(x), T(-2));
    EXPECT_EQ(fxp::trunc(x), T(-2)); EXPECT_EQ(fxp::round(x), T(-3));
    EXPECT_EQ(fxp::nearbyint(x), T(-2)); EXPECT_EQ(fxp::rint(x), T(-2));
    EXPECT_EQ(fxp::ceil(T::max()), T::max());
    EXPECT_EQ(fxp::floor(T::min()), T::min());
    T integral;
    EXPECT_EQ(fxp::modf(x, &integral), T::from_string("-0.5")); EXPECT_EQ(integral, T(-2));
    EXPECT_EQ(fxp::copysign(T::min(), T(-1)), T::min());
    EXPECT_EQ(fxp::copysign(T::min(), T(1)), T::max());
}
TYPED_TEST(MathTest, IdentitiesAndRoots) {
    using T = TypeParam;
    EXPECT_EQ(fxp::sin(T{}), T{}); EXPECT_EQ(fxp::cos(T{}), T(1)); EXPECT_EQ(fxp::tan(T{}), T{});
    EXPECT_EQ(fxp::asin(T{}), T{}); EXPECT_EQ(fxp::atan(T{}), T{});
    EXPECT_EQ(fxp::acos(T(1)), T{}); EXPECT_EQ(fxp::acos(T(-1)), fxp::pi_v<T>);
    EXPECT_EQ(fxp::atan2(T{}, T(-1)), fxp::pi_v<T>);
    EXPECT_EQ(fxp::exp(T{}), T(1)); EXPECT_EQ(fxp::exp2(T{}), T(1)); EXPECT_EQ(fxp::expm1(T{}), T{});
    EXPECT_EQ(fxp::log(T(1)), T{}); EXPECT_EQ(fxp::log2(T(8)), T(3)); EXPECT_EQ(fxp::log10(T(1000)), T(3));
    EXPECT_EQ(fxp::log1p(T{}), T{});
    EXPECT_EQ(fxp::sqrt(T(4)), T(2)); EXPECT_EQ(fxp::cbrt(T(-8)), T(-2));
    EXPECT_EQ(fxp::hypot(T(3), T(4)), T(5)); EXPECT_EQ(fxp::hypot(T(2), T(3), T(6)), T(7));
    EXPECT_EQ(fxp::hypot(T::min(), T::min()), T::max());
    EXPECT_EQ(fxp::hypot(T::min(), T::min(), T::min()), T::max());
    EXPECT_EQ(fxp::pow(T(-2), 3), T(-8)); EXPECT_EQ(fxp::pow(T(2), -3), T::from_string("0.125"));
    EXPECT_EQ(fxp::pow(T{}, 0), T(1)); EXPECT_EQ(fxp::pow(T(-2), T(4)), T(16));
    EXPECT_EQ(fxp::pow(T(4), T::from_string("0.5")), T(2));
    EXPECT_EQ(fxp::pow(T(-1), std::numeric_limits<std::int64_t>::min()), T(1));
    EXPECT_EQ(fxp::exp(T::max()), T::max()); EXPECT_EQ(fxp::exp(T::min()), T{});
    EXPECT_EQ(fxp::expm1(T::min()), T(-1)); EXPECT_EQ(fxp::expm1(T(60)), T::max());
    EXPECT_EQ(fxp::exp2(T(63)), T::max());
    EXPECT_EQ(fxp::exp2(T(-static_cast<int>(T::fractional_bits) - 1)), T{});
    EXPECT_EQ(fxp::sqrt(T::epsilon()).raw_value(), INT64_C(1) << (T::fractional_bits / 2));
}
TYPED_TEST(MathTest, DomainErrors) {
    using T = TypeParam;
    EXPECT_THROW((void)fxp::sqrt(T(-1)), std::domain_error);
    EXPECT_THROW((void)fxp::log(T{}), std::domain_error);
    EXPECT_THROW((void)fxp::log2(T(-1)), std::domain_error);
    EXPECT_THROW((void)fxp::log10(T{}), std::domain_error);
    EXPECT_THROW((void)fxp::log1p(T(-1)), std::domain_error);
    EXPECT_THROW((void)fxp::asin(T(2)), std::domain_error);
    EXPECT_THROW((void)fxp::acos(T(-2)), std::domain_error);
    EXPECT_THROW((void)fxp::atan2(T{}, T{}), std::domain_error);
    EXPECT_THROW((void)fxp::pow(T(-2), T::from_string("0.5")), std::domain_error);
    EXPECT_THROW((void)fxp::pow(T{}, -1), std::domain_error);
    EXPECT_THROW((void)fxp::remainder(T(1), T{}), std::domain_error);
    EXPECT_THROW((void)fxp::remquo(T(1), T(2), nullptr), std::invalid_argument);
    EXPECT_THROW((void)fxp::modf(T(1), static_cast<T*>(nullptr)), std::invalid_argument);
}
TYPED_TEST(MathTest, ScalingAndClassification) {
    using T = TypeParam;
    EXPECT_EQ(fxp::ldexp(T(3), 2), T(12));
    EXPECT_EQ(fxp::ldexp(T::from_raw(5), -1), T::from_raw(2));
    EXPECT_EQ(fxp::ldexp(T(-1), std::numeric_limits<int>::max()), T::min());
    EXPECT_EQ(fxp::ldexp(T(1), std::numeric_limits<int>::min()), T{});
    EXPECT_EQ(fxp::ldexp(T{}, 100), T{});
    int e = 0;
    EXPECT_EQ(fxp::frexp(T(3), &e), T::from_string("0.75")); EXPECT_EQ(e, 2);
    EXPECT_EQ(fxp::frexp(T::min(), &e), T::from_string("-0.5")); EXPECT_EQ(e, 64 - static_cast<int>(T::fractional_bits));
    EXPECT_EQ(fxp::nextafter(T::max(), T::min()), T::from_raw(INT64_MAX - 1));
    EXPECT_EQ(fxp::nextafter(T::min(), T::max()), T::from_raw(INT64_MIN + 1));
    EXPECT_EQ(fxp::nextafter(T::min(), T::min()), T::min());
    EXPECT_TRUE(fxp::isfinite(T::max())); EXPECT_FALSE(fxp::isnan(T{})); EXPECT_FALSE(fxp::isinf(T{}));
    EXPECT_TRUE(fxp::signbit(T(-1)));
}
