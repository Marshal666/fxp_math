#include <fxp/fxp.hpp>
#include <gtest/gtest.h>
#include <sstream>
#include <unordered_set>

using namespace fxp::literals;
using fxp::real;
using fxp::fine;

static_assert((1.2_r).raw_value() == 78643);
static_assert((4.00002345_fine).raw_value() == INT64_C(17179969901));
static_assert(1'234.5_r == real::from_raw(80904192));
static_assert((1e-3_r).raw_value() == 66);
static_assert(std::is_same_v<decltype(real{} + fine{}), fine>);
static_assert(std::is_same_v<decltype(fine{} * real{}), fine>);
static_assert(std::is_same_v<decltype(2 / fine{}), fine>);
static_assert(std::is_same_v<std::common_type_t<real, fine>, fine>);
static_assert(!std::is_constructible_v<real, float> && !std::is_convertible_v<double, real>);
static_assert(!std::is_assignable_v<real&, double> && !std::is_convertible_v<real, double>);
static_assert(std::is_trivially_copyable_v<real> && sizeof(real) == 8);
static_assert(real(123).raw_value() == 8060928);
static_assert(real::from_string("0.00000762939453125").raw_value() == 0);
static_assert(real::from_string("0.00002288818359375").raw_value() == 2);

template<class T> class FixedTest : public ::testing::Test {};
using Formats = ::testing::Types<real, fine>;
TYPED_TEST_SUITE(FixedTest, Formats, );

TYPED_TEST(FixedTest, SaturationAndIntegerConversions) {
    using T = TypeParam;
    EXPECT_EQ(T::max() + T::epsilon(), T::max());
    EXPECT_EQ(T::min() - T::epsilon(), T::min());
    EXPECT_EQ(T::min() + T::max(), T::from_raw(-1));
    EXPECT_EQ(T::min() - T::min(), T{});
    EXPECT_EQ(-T::min(), T::max());
    EXPECT_EQ(fxp::abs(T::min()), T::max());
    EXPECT_EQ(T(std::numeric_limits<std::uint64_t>::max()), T::max());
    EXPECT_EQ(T(std::numeric_limits<std::int64_t>::min()), T::min());
    EXPECT_EQ(T::from_string("2.5").template to_integer<>(), 2);
    EXPECT_EQ(T::from_string("3.5").template to_integer<>(), 4);
    EXPECT_EQ(T::from_string("-3.5").template to_integer<>(), -4);
    EXPECT_EQ(T(-1000).template to_integer<std::int8_t>(), -128);
    EXPECT_EQ(T(1000).template to_integer<std::uint8_t>(), 255);
    EXPECT_EQ(T(-3).template to_integer<std::uint64_t>(), 0U);
    EXPECT_TRUE(static_cast<bool>(T::epsilon()));
}
TYPED_TEST(FixedTest, ArithmeticRoundingAndSignedExtremes) {
    using T = TypeParam;
    const auto half = T::from_raw(static_cast<std::int64_t>(T::scale / 2));
    EXPECT_EQ(T::epsilon() * half, T{});
    EXPECT_EQ(T::from_raw(3) * half, T::from_raw(2));
    EXPECT_EQ(T::from_raw(-3) * half, T::from_raw(-2));
    EXPECT_EQ(T::from_raw(5) / 2, T::from_raw(2));
    EXPECT_EQ(T::from_raw(-7) / 2, T::from_raw(-4));
    EXPECT_EQ(T::min() / -1, T::max());
    EXPECT_EQ(T::min() * -1, T::max());
    EXPECT_EQ(T::min() / 1, T::min());
    EXPECT_EQ(T::min() % -1, T{});
    EXPECT_EQ(T::min() / T::min(), T(1));
    EXPECT_THROW((void)(T(1) / 0), std::domain_error);
    EXPECT_THROW((void)(T(1) % 0), std::domain_error);
    T x = 3;
    EXPECT_EQ(x++, T(3)); EXPECT_EQ(++x, T(5));
    x *= 2; x /= 4; x += 1; x -= 2; x %= 1;
    EXPECT_EQ(x, half);
}
TYPED_TEST(FixedTest, ExactStringsAndMalformedInputs) {
    using T = TypeParam;
    for (const auto raw : {INT64_C(0), INT64_C(1), INT64_C(-1), INT64_C(123456789), INT64_MIN, INT64_MAX}) {
        const auto x = T::from_raw(raw);
        EXPECT_EQ(T::from_string(x.to_string()), x);
        EXPECT_EQ(T(std::string(x)), x);
    }
    EXPECT_EQ(T::from_string("1e10000000000"), T::max());
    EXPECT_EQ(T::from_string("-1e10000000000"), T::min());
    EXPECT_EQ(T::from_string("1e-10000000000"), T{});
    EXPECT_EQ(T::from_string("0e99999"), T{});
    EXPECT_EQ(T::from_string("+.5"), T::from_string("0.5"));
    for (const auto text : {"", "+", ".", "1e", "1e-", "NaN", "inf", "0x1", " 1", "1 ", "1.2.3", "1e1.0", "1''2"})
        EXPECT_THROW((void)T::from_string(text), std::invalid_argument) << text;
    std::istringstream good("1.25"), bad("oops");
    T value;
    good >> value; EXPECT_EQ(value, T::from_string("1.25"));
    bad >> value; EXPECT_TRUE(bad.fail()); EXPECT_EQ(value, T::from_string("1.25"));
}
TEST(Decimal, StickyDigitsAcrossExactMidpoints) {
    EXPECT_EQ(real::from_string("0.00000762939453125000000000000000001").raw_value(), 1);
    EXPECT_EQ(real::from_string("0.00000762939453124999999999999999999").raw_value(), 0);
    EXPECT_EQ(fine::from_string("0.000000000116415321826934814453125").raw_value(), 0);
    EXPECT_EQ(fine::from_string("0.000000000116415321826934814453125000001").raw_value(), 1);
    EXPECT_EQ(fine::from_string("-0.000000000116415321826934814453125000001").raw_value(), -1);
}
TEST(Conversions, MixedPromotionAndRange) {
    EXPECT_EQ(fine(1.5_r), 1.5_fine);
    EXPECT_EQ(real(1.5_fine), 1.5_r);
    EXPECT_EQ(real(fine::from_raw(32768)), real{});
    EXPECT_EQ(real(fine::from_raw(98304)), real::from_raw(2));
    EXPECT_EQ(fine(real::max()), fine::max());
    EXPECT_EQ(fine(real::min()), fine::min());
    EXPECT_EQ(real::max() + fine::min(), fine::from_raw(-1));
    EXPECT_EQ(real::max(), fine::max()); // Comparisons also use promotion first.
    EXPECT_EQ(1.25_r + 0.00000001_fine, 1.25_fine + 0.00000001_fine);
    EXPECT_EQ(3 + 2_r, 5_r); EXPECT_EQ(3 - 2_r, 1_r);
    std::unordered_set<real> values{1_r, 2_r, 1_r};
    EXPECT_EQ(values.size(), 2U);
}
