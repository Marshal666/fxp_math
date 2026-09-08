#include "reference_ops.hpp"
#include <gtest/gtest.h>

TEST(Reference, ExactCrossPlatformResults) {
    const auto rows = reference::read(std::string(FXP_REFERENCE_DIR) + "/results.csv");
    const auto oracle = reference::read(std::string(FXP_REFERENCE_DIR) + "/oracle.csv");
    ASSERT_GT(rows.size(), 10000U);
    ASSERT_EQ(rows.size(), oracle.size());
    for (std::size_t i = 0; i < rows.size(); ++i) {
        ASSERT_EQ(rows[i].format, oracle[i].format);
        ASSERT_EQ(rows[i].operation, oracle[i].operation);
        ASSERT_EQ(rows[i].a, oracle[i].a);
        ASSERT_EQ(rows[i].b, oracle[i].b);
    }
    for (const auto& row : rows) {
        const auto actual = reference::evaluate(row);
        EXPECT_EQ(actual.value, row.result) << row.format << ':' << row.operation << '(' << row.a << ',' << row.b << ')';
        EXPECT_EQ(actual.auxiliary, row.auxiliary) << row.operation;
    }
}
TEST(Reference, IndependentIntegerAndHighPrecisionOracles) {
    const auto rows = reference::read(std::string(FXP_REFERENCE_DIR) + "/oracle.csv");
    ASSERT_GT(rows.size(), 10000U);
    for (const auto& row : rows) {
        const auto actual = reference::evaluate(row);
        if (!row.tolerance || row.result == "domain" || row.result == "invalid" || actual.value == "domain" || actual.value == "invalid") {
            EXPECT_EQ(actual.value, row.result) << row.format << ':' << row.operation << '(' << row.a << ',' << row.b << ')';
        } else {
            const auto a = static_cast<std::int64_t>(std::stoll(actual.value));
            const auto b = static_cast<std::int64_t>(std::stoll(row.result));
            const auto error = a > b ? static_cast<std::uint64_t>(a) - static_cast<std::uint64_t>(b)
                : static_cast<std::uint64_t>(b) - static_cast<std::uint64_t>(a);
            EXPECT_LE(error, row.tolerance) << row.format << ':' << row.operation << '(' << row.a << ',' << row.b << ") actual=" << a << " oracle=" << b;
        }
        EXPECT_EQ(actual.auxiliary, row.auxiliary) << row.operation;
    }
}
