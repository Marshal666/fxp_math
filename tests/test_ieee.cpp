#include <fxp/fxp.hpp>
#include <gtest/gtest.h>
using namespace fxp::literals;
TEST(IEEE754, KnownBitsAndEndianOrder) {
    constexpr auto one = fxp::from_ieee754_binary32_bits<>(UINT32_C(0x3f800000));
    static_assert(one == 1_r);
    EXPECT_EQ(fxp::from_ieee754_binary64_bits<fxp::fine>(UINT64_C(0xc004000000000000)), -2.5_fine);
    EXPECT_EQ(fxp::to_ieee754_binary32_bits(-2.5_r), UINT32_C(0xc0200000));
    EXPECT_EQ(fxp::to_ieee754_binary64_bits(1_r), UINT64_C(0x3ff0000000000000));
    const std::array<std::uint8_t, 4> big{0x3f, 0xc0, 0, 0}, little{0, 0, 0xc0, 0x3f};
    EXPECT_EQ(fxp::from_ieee754_binary32_bytes<>(big, fxp::byte_order::big_endian), 1.5_r);
    EXPECT_EQ(fxp::from_ieee754_binary32_bytes<>(little, fxp::byte_order::little_endian), 1.5_r);
    EXPECT_EQ(fxp::to_ieee754_binary32_bytes(1.5_r, fxp::byte_order::big_endian), big);
    EXPECT_EQ(fxp::to_ieee754_binary32_bytes(1.5_r, fxp::byte_order::little_endian), little);
    for (auto order : {fxp::byte_order::big_endian, fxp::byte_order::little_endian}) {
        auto bytes = fxp::to_ieee754_binary64_bytes(4.00002345_fine, order);
        EXPECT_EQ(fxp::from_ieee754_binary64_bytes<fxp::fine>(bytes, order), 4.00002345_fine);
    }
}
TEST(IEEE754, SpecialValuesAndTies) {
    EXPECT_EQ(fxp::from_ieee754_binary32_bits<>(UINT32_C(0x80000000)), 0_r);
    EXPECT_EQ(fxp::from_ieee754_binary32_bits<>(1), 0_r);
    EXPECT_EQ(fxp::from_ieee754_binary64_bits<fxp::fine>(UINT64_C(0x000fffffffffffff)), 0_fine);
    EXPECT_EQ(fxp::from_ieee754_binary32_bits<>(UINT32_C(0x7f800000)), fxp::real::max());
    EXPECT_EQ(fxp::from_ieee754_binary64_bits<>(UINT64_C(0xfff0000000000000)), fxp::real::min());
    for (auto bits : {UINT32_C(0x7fc00000), UINT32_C(0x7f800001), UINT32_C(0xff800001)})
        EXPECT_THROW((void)fxp::from_ieee754_binary32_bits<>(bits), std::domain_error);
    EXPECT_THROW((void)fxp::from_ieee754_binary64_bits<>(UINT64_C(0x7ff0000000000001)), std::domain_error);
    EXPECT_EQ(fxp::from_ieee754_binary32_bits<>(UINT32_C(0x37000000)).raw_value(), 0); // 2^-17
    EXPECT_EQ(fxp::from_ieee754_binary32_bits<>(UINT32_C(0x37c00000)).raw_value(), 2); // 3 * 2^-17
    EXPECT_EQ(fxp::to_ieee754_binary32_bits(fxp::fine::from_raw((INT64_C(1) << 32) + 256)), UINT32_C(0x3f800000));
    EXPECT_EQ(fxp::to_ieee754_binary32_bits(fxp::fine::from_raw((INT64_C(1) << 32) + 768)), UINT32_C(0x3f800002));
}
