#include <fxp/fxp.hpp>
#include <gtest/gtest.h>

TEST(Tangent, LargeAnglesNearPolesKeepSignAndDoNotThrow) {
    const auto real = fxp::real::from_raw(INT64_C(2949191698433286955));
    EXPECT_LT(fxp::tan(real).raw_value(), 0);
    EXPECT_GT(fxp::tan(-real).raw_value(), 0);
    const auto fine = fxp::fine::from_raw(INT64_C(1815789637366200116));
    EXPECT_EQ(fxp::tan(fine), fxp::fine::min());
    EXPECT_EQ(fxp::tan(-fine), fxp::fine::max());
}
