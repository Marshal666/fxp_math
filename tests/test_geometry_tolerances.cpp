#include <fxp/geometry.hpp>
#include <gtest/gtest.h>
#include <algorithm>
#include <array>
#include <fstream>
#include <sstream>
#include <string>

using namespace fxp;
template <class T> class GeometryTolerance : public ::testing::Test {};
using ToleranceFormats = ::testing::Types<real, fine>;
TYPED_TEST_SUITE(GeometryTolerance, ToleranceFormats, );

TYPED_TEST(GeometryTolerance, NormalProjectionRetainsItsDirectionAtSmallScales) {
    using T = TypeParam;
    for (const char *decimal : {"0.0002", "0.001", "0.002", "0.005", "0.01", "0.1"}) {
        const auto t = T::from_string(decimal);
        T phi, theta;
        get_angles(vec3<T>(1, t, t), &phi, &theta);
        // Equal positive Y and Z give exactly a 45-degree projected angle.
        EXPECT_LE(abs(phi + pi_v<T> / 4), T::epsilon()) << decimal;
        get_angles(vec3<T>(-t, 1, -t), &phi, &theta);
        EXPECT_LE(abs(theta + 3 * pi_v<T> / 4), T::epsilon() * 2) << decimal;
    }
}

TYPED_TEST(GeometryTolerance, SmallRotationsProduceUnitAxesAndRoundTrip) {
    using T = TypeParam;
    for (const char *decimal : {"0.0005", "0.001", "0.003", "0.005", "0.01", "0.1"}) {
        const auto input = T::from_string(decimal);
        quat<T> q(input, vec3<T>(0, 0, 1));
        T angle, scalar_angle, x, y, z;
        vec3<T> axis;
        q.decomp_angle_axis(&angle, &axis);
        q.decomp_angle_axis(&scalar_angle, &x, &y, &z);
        EXPECT_EQ(axis, vec3<T>(0, 0, 1)) << decimal;
        EXPECT_LE(abs(angle - input), T::epsilon() * 3) << decimal;
        EXPECT_EQ(scalar_angle, angle);
        EXPECT_EQ(vec3<T>(x, y, z), axis);
        const quat<T> restored(angle, axis);
        for (std::size_t i = 0; i < 4; ++i)
            EXPECT_LE(abs(restored.get_internal_vector()[i] - q.get_internal_vector()[i]), T::epsilon() * 2);
    }
}

TYPED_TEST(GeometryTolerance, TinyTrianglesAndDegenerateSegmentsRetainExactIncidence) {
    using T = TypeParam;
    using V = vec2<T>;
    const auto e = T::epsilon();
    for (const auto base : {T{}, T(1), T(-10), T(1000)}) {
        for (std::int64_t raw : {1, 2, 3, 64, 181, 256, 1024}) {
            const auto step = T::from_raw(raw);
            const V a(base, base), b(base + step, base + step), c(base + 2 * step, base + 2 * step);
            EXPECT_TRUE(is_point_inside_triangle(a, c, a, b)) << base << ':' << raw;
            EXPECT_TRUE(is_point_inside_triangle(c, a, c, b)) << base << ':' << raw;
            // An off-line point is outside regardless of the endpoint tolerance.
            EXPECT_FALSE(is_point_inside_triangle(a, b, c, V(b.x, b.y + e))) << base << ':' << raw;
            const V p(base + 2 * step, base), q(base, base + 2 * step);
            EXPECT_TRUE(is_point_inside_triangle(a, p, q, b)) << base << ':' << raw;
            EXPECT_FALSE(is_point_inside_triangle(a, p, q, V(b.x + e, b.y))) << base << ':' << raw;
        }
    }
}

TYPED_TEST(GeometryTolerance, DegenerateEndpointToleranceHasAnExplicitBoundary) {
    using T = TypeParam;
    using V = vec2<T>;
    // The source's 1e-6 distance slack applies after exact collinearity checks.
    const auto inside_raw = static_cast<std::int64_t>(T::scale / UINT64_C(1000000));
    const auto outside_raw = inside_raw + 1;
    const V a(0, 0), b(1, 0), c(2, 0);
    EXPECT_TRUE(is_point_inside_triangle(a, b, c, V(-T::from_raw(inside_raw), 0)));
    EXPECT_FALSE(is_point_inside_triangle(a, b, c, V(-T::from_raw(outside_raw), 0)));
    EXPECT_TRUE(is_point_inside_triangle(a, b, c, V(T(2) + T::from_raw(inside_raw), 0)));
    EXPECT_FALSE(is_point_inside_triangle(a, b, c, V(T(2) + T::from_raw(outside_raw), 0)));
    EXPECT_TRUE(is_point_inside_triangle(a, a, a, a));
    EXPECT_FALSE(is_point_inside_triangle(a, a, a, V(T::from_raw(outside_raw), 0)));
}

TYPED_TEST(GeometryTolerance, TriangleIncidenceMatchesIntegerGeometryAcrossTranslations) {
    using T = TypeParam;
    using V = vec2<T>;
    struct Point {
        std::int64_t x, y;
    };
    const auto area = [](Point a, Point b, Point c) {
        // A bounded, ordinary int64 shoelace oracle, independent of the wide
        // signed-product implementation. All coordinates here are in [-2,2].
        return a.x * (b.y - c.y) + b.x * (c.y - a.y) + c.x * (a.y - b.y);
    };
    std::array<Point, 9> vertices{};
    for (std::size_t i = 0; i < vertices.size(); ++i)
        vertices[i] = {static_cast<std::int64_t>(i % 3) - 1, static_cast<std::int64_t>(i / 3) - 1};
    for (const auto offset : {T{}, T::from_raw(INT64_MIN + 4), T::from_raw(INT64_MAX - 4)}) {
        const auto translate = [&](Point p) {
            return V(offset + T::from_raw(p.x), offset + T::from_raw(p.y));
        };
        for (const auto a : vertices)
            for (const auto b : vertices)
                for (const auto c : vertices) {
                    const auto orientation = area(a, b, c);
                    if (!orientation)
                        continue; // Degenerate cases have their separate endpoint-slack tests.
                    for (std::int64_t x = -2; x <= 2; ++x)
                        for (std::int64_t y = -2; y <= 2; ++y) {
                            const Point p{x, y};
                            const bool expected = orientation * area(p, a, b) >= 0 &&
                                                  orientation * area(p, b, c) >= 0 &&
                                                  orientation * area(p, c, a) >= 0;
                            ASSERT_EQ(is_point_inside_triangle(translate(a), translate(b), translate(c),
                                                               translate(p)),
                                      expected)
                                << offset << ':' << a.x << ',' << a.y << ';' << b.x << ',' << b.y << ';'
                                << c.x << ',' << c.y << ';' << x << ',' << y;
                        }
                }
    }
    // Spanning both signed storage limits also exercises 65-bit differences.
    const V a(T::min(), T::min()), b(T::max(), T::min()), c(T::min(), T::max());
    EXPECT_TRUE(is_point_inside_triangle(a, b, c, V(-T::epsilon(), 0)));
    EXPECT_FALSE(is_point_inside_triangle(a, b, c, V{}));
    EXPECT_TRUE(is_point_inside_triangle(a, c, b, V(-T::epsilon(), 0)));
    EXPECT_FALSE(is_point_inside_triangle(a, c, b, V{}));
}

template <class T>
void check_geometry_oracle(const std::string &operation, const std::array<std::int64_t, 8> &row) {
    std::array<T, 4> actual{};
    if (operation == "normal") {
        get_angles(vec3<T>(T::from_raw(row[0]), T::from_raw(row[1]), T::from_raw(row[2])), &actual[0],
                   &actual[1]);
    } else {
        ASSERT_EQ(operation, "quaternion");
        quat<T> q;
        q.from_components(T::from_raw(row[0]), T::from_raw(row[1]), T::from_raw(row[2]), T::from_raw(row[3]));
        q.decomp_angle_axis(&actual[0], &actual[1], &actual[2], &actual[3]);
        vec3<T> axis;
        T angle;
        q.decomp_angle_axis(&angle, &axis);
        EXPECT_EQ(angle, actual[0]);
        EXPECT_EQ(axis, vec3<T>(actual[1], actual[2], actual[3]));
        EXPECT_LE(abs(length(axis) - T(1)), T::epsilon() * 2);
    }
    for (std::size_t i = 0; i < 4; ++i) {
        // One output rounding for angles/axes; quaternion angle also rounds its
        // vector length and doubles the rounded half-angle (at most two raw units).
        const auto budget = operation == "quaternion" && i == 0 ? 2 : 1;
        EXPECT_LE(abs(actual[i] - T::from_raw(row[4 + i])), T::epsilon() * budget) << "component " << i;
    }
}

TEST(GeometryToleranceOracle, IndependentHighPrecisionResults) {
    std::ifstream file(std::string(FXP_REFERENCE_DIR) + "/geometry_tolerance_oracle.csv");
    ASSERT_TRUE(file);
    std::string line;
    std::size_t count = 0;
    while (std::getline(file, line)) {
        if (line.empty() || line[0] == '#')
            continue;
        SCOPED_TRACE(line);
        std::replace(line.begin(), line.end(), ',', ' ');
        std::istringstream in(line);
        unsigned format;
        std::string operation;
        std::array<std::int64_t, 8> row{};
        ASSERT_TRUE(in >> format >> operation);
        for (auto &field : row)
            ASSERT_TRUE(in >> field);
        if (format == 16)
            check_geometry_oracle<real>(operation, row);
        else {
            ASSERT_EQ(format, 32U);
            check_geometry_oracle<fine>(operation, row);
        }
        ++count;
    }
    EXPECT_EQ(count, 1448U);
}
