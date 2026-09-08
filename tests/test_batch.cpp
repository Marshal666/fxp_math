#include <fxp/fxp.hpp>
#include <gtest/gtest.h>
#include <vector>
template<class T> class BatchTest : public ::testing::Test {};
using BatchFormats = ::testing::Types<fxp::real, fxp::fine>;
TYPED_TEST_SUITE(BatchTest, BatchFormats, );
TYPED_TEST(BatchTest, ScalarAgreementAndAliasing) {
    using T = TypeParam;
    std::vector<T> a(4099), b(4099), expected(4099), output(4099);
    std::uint64_t state = 987654321;
    for (std::size_t i = 0; i < a.size(); ++i) {
        state ^= state << 13; state ^= state >> 7; state ^= state << 17;
        a[i] = T::from_raw(fxp::detail::signed_magnitude({0, state >> 1}, (state & 1) != 0));
        state ^= state << 13; state ^= state >> 7; state ^= state << 17;
        b[i] = T::from_raw(fxp::detail::signed_magnitude({0, state >> 1}, (state & 1) != 0));
    }
    a[1] = T::min(); b[1] = T::min(); a[2] = T::max(); b[2] = T::max();
    for (const auto count : {std::size_t{0}, std::size_t{1}, std::size_t{2}, std::size_t{4097}}) {
        fxp::batch::add(a.data() + 1, b.data() + 1, expected.data() + 1, count, fxp::batch::backend::scalar);
        fxp::batch::add(a.data() + 1, b.data() + 1, output.data() + 1, count);
        EXPECT_EQ(expected, output);
        fxp::batch::subtract(a.data() + 1, b.data() + 1, expected.data() + 1, count, fxp::batch::backend::scalar);
        fxp::batch::subtract(a.data() + 1, b.data() + 1, output.data() + 1, count);
        EXPECT_EQ(expected, output);
    }
    fxp::batch::add(a.data(), b.data(), expected.data(), a.size(), fxp::batch::backend::scalar);
    auto inplace = a;
    fxp::batch::add(inplace.data(), b.data(), inplace.data(), a.size()); EXPECT_EQ(inplace, expected);
    inplace = b;
    fxp::batch::add(a.data(), inplace.data(), inplace.data(), a.size()); EXPECT_EQ(inplace, expected);
    fxp::batch::subtract(a.data(), a.data(), a.data(), a.size());
    for (auto v : a) EXPECT_EQ(v, T{});
    fxp::batch::add<T::fractional_bits>(nullptr, nullptr, nullptr, 0);
    const T x[]{T(2), T(3)}, y[]{T(4), T(6)}; T z[2];
    fxp::batch::multiply(x, y, z, 2); EXPECT_EQ(z[0], T(8)); EXPECT_EQ(z[1], T(18));
    fxp::batch::divide(y, x, z, 2); EXPECT_EQ(z[0], T(2)); EXPECT_EQ(z[1], T(2));
}
