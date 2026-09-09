#include <fxp/geometry_misc.hpp>
#include <gtest/gtest.h>
#include <set>
#include <vector>
#include <fstream>
#include "geometry_reference.hpp"

using namespace fxp;
static_assert(std::is_same_v<decltype(real3{} + fine3{}), fine3>);
static_assert(std::is_same_v<decltype(real3{} * fine{}), fine3>);
static_assert(std::is_same_v<decltype(real3{} * fine3{}), fine>);
static_assert(std::is_same_v<std::decay_t<decltype(mat4<real>{} * mat4<fine>{})>, mat4<fine>>);
static_assert(!std::is_constructible_v<real3, float, float, float>);
static_assert(!std::is_constructible_v<quat<real>, double, real3>);
static_assert(sizeof(real3) == 3 * sizeof(real));
static_assert(sizeof(mat4<fine>) == 16 * sizeof(fine));
static_assert(std::is_trivially_copyable_v<real4>);
static_assert(axis3_x_v<real>.x == 1);

template <class T> class Geometry : public ::testing::Test {};
using GeometryFormats = ::testing::Types<real, fine>;
TYPED_TEST_SUITE(Geometry, GeometryFormats, );
template <class T> void near_raw(T a, T b, std::int64_t tolerance = 8) {
    EXPECT_LE(fxp::abs(a - b).raw_value(), tolerance) << a << " versus " << b;
}
template <class T, std::size_t N>
void near_vector(const vector<T, N> &a, const vector<T, N> &b, std::int64_t tolerance = 8) {
    for (std::size_t i = 0; i < N; ++i)
        near_raw(a[i], b[i], tolerance);
}
TYPED_TEST(Geometry, VectorsAndNormalization) {
    using T = TypeParam;
    using V = vec3<T>;
    V a(1, 2, 3), b(-3, 4, 1);
    EXPECT_EQ(a + b, V(-2, 6, 4));
    EXPECT_EQ(a - b, V(4, -2, 2));
    EXPECT_EQ(a * b, T(8));
    EXPECT_EQ(a ^ b, V(-10, -10, 10));
    EXPECT_EQ(cross(vec2<T>(2, 3), vec2<T>(5, 7)), T(-1));
    vec2<T> c(2, 3);
    c ^= c;
    EXPECT_EQ(c, vec2<T>(-5, 12));
    EXPECT_EQ(length(V(2, 3, 6)), T(7));
    EXPECT_EQ(length_squared(V(2, 3, 6)), T(49));
    V zero;
    EXPECT_FALSE(normalize(&zero));
    EXPECT_EQ(zero, V{});
    V axis(T::epsilon(), 0, 0);
    EXPECT_TRUE(normalize(&axis));
    EXPECT_EQ(axis, V(1, 0, 0));
    vec2<T> tiny(T::epsilon(), T::epsilon());
    normalize(&tiny);
    near_raw(tiny.x, fxp::sqrt(T::from_string("0.5")), 1);
    EXPECT_EQ(tiny.x, tiny.y);
    vec4<T> huge(T::min(), T::min(), T::min(), T::min());
    EXPECT_EQ(length(huge), T::max());
    EXPECT_TRUE(normalize(&huge));
    EXPECT_EQ(huge, vec4<T>(T::from_string("-0.5"), T::from_string("-0.5"), T::from_string("-0.5"),
                            T::from_string("-0.5")));
    a.lerp(T::from_string("0.5"), a, b);
    EXPECT_EQ(a, V(-1, 3, 2));
    a.displace(a, T(2));
    EXPECT_EQ(a, V(-3, 9, 6));
    a.negate(a);
    EXPECT_EQ(a, V(3, -9, -6));
    a.r() = 7;
    EXPECT_EQ(a.x, T(7));
    a[1] = 8;
    EXPECT_EQ(a.g(), T(8));
    EXPECT_THROW(a[3], std::out_of_range);
    EXPECT_THROW((void)(a / T{}), std::domain_error);
    EXPECT_THROW(normalize(static_cast<V *>(nullptr)), std::invalid_argument);
    vec4<T> extended(V(1, 2, 3), T(4));
    EXPECT_EQ(extended.w, T(4));
    EXPECT_EQ(vec3_hash<T>{}(V(1, 2, 3)), vec3_hash<T>{}(V(1, 2, 3)));
}
TYPED_TEST(Geometry, MatrixInverseAliasingAndTransforms) {
    using T = TypeParam;
    using M = mat4<T>;
    using V = vec3<T>;
    M unit;
    identity(&unit);
    M m = unit;
    m(0, 0) = 2;
    m(1, 1) = 4;
    m(2, 2) = 8;
    m(0, 3) = 3;
    m(1, 3) = -2;
    m(2, 3) = 5;
    V point(1, 2, 3);
    m.rotate_h_vector(&point, point);
    EXPECT_EQ(point, V(5, 6, 29));
    M inv;
    ASSERT_TRUE(invert(&inv, m));
    EXPECT_EQ(m * inv, unit);
    EXPECT_EQ(inv * m, unit);
    M alias = m;
    ASSERT_TRUE(invert(&alias, alias));
    EXPECT_EQ(alias, inv);
    alias = m;
    ASSERT_TRUE(alias.homogeneous_inverse(alias));
    EXPECT_EQ(alias, inv);
    M product = m * m;
    alias = m;
    multiply(&alias, alias, alias);
    EXPECT_EQ(alias, product);
    alias = m;
    transpose(&alias, alias);
    M trans;
    transpose(&trans, m);
    EXPECT_EQ(alias, trans);
    multiply_scale(&alias, m, T(2), T(3), T(4));
    EXPECT_EQ(alias(1, 1), T(12));
    EXPECT_EQ(alias(0, 3), T(3));
    M zero;
    alias = unit;
    EXPECT_FALSE(invert(&alias, zero));
    EXPECT_EQ(alias, unit);
    EXPECT_FALSE(alias.homogeneous_inverse(zero));
    EXPECT_EQ(alias, unit);
    EXPECT_EQ(det(m), T(64));
    EXPECT_EQ(det(T(1), T(2), T(3), T(4)), T(-2));
    transform_pair<T> p{m, inv};
    EXPECT_EQ((p * p).forward, product);
    EXPECT_EQ((p * p).backward, inv * inv);
    vec4<T> hv(1, 2, 3, 1);
    m.rotate_h_vector(&hv, hv);
    EXPECT_EQ(hv, vec4<T>(5, 6, 29, 1));
    m.rotate_h_direction(&hv, V(1, 2, 3));
    EXPECT_EQ(hv, vec4<T>(2, 8, 24, 0));
}
TYPED_TEST(Geometry, QuaternionConventionsAndDecomposition) {
    using T = TypeParam;
    using Q = quat<T>;
    using V = vec3<T>;
    Q id;
    EXPECT_EQ(id.get_internal_vector(), vec4<T>(0, 0, 0, 1));
    Q z(pi_v<T> / 2, axis3_z_v<T>), x(pi_v<T> / 2, axis3_x_v<T>);
    near_vector(z.rotate(V(1, 0, 0)), V(0, 1, 0), 12);
    Q composition = z;
    composition *= x;
    EXPECT_EQ(composition.get_internal_vector(), (x * z).get_internal_vector());
    Q invx;
    invx.unit_inverse(x);
    EXPECT_EQ((z / x).get_internal_vector(), (invx * z).get_internal_vector());
    EXPECT_EQ((z / id).get_internal_vector(), z.get_internal_vector());
    Q nonunit;
    nonunit.from_components(T{}, T{}, T{}, T(2));
    Q inverse;
    ASSERT_TRUE(inverse.inverse(nonunit));
    EXPECT_EQ(inverse.get_internal_vector(), vec4<T>(0, 0, 0, T::from_string("0.5")));
    EXPECT_EQ((nonunit * inverse).get_internal_vector(), id.get_internal_vector());
    Q zero(vec4<T>{});
    EXPECT_FALSE(inverse.inverse(zero));
    Q s;
    s.slerp(T{}, id, z);
    EXPECT_EQ(s.get_internal_vector(), id.get_internal_vector());
    s.slerp(T(1), id, z);
    near_vector(s.get_internal_vector(), z.get_internal_vector(), 3);
    s.slerp(T::from_string("0.5"), z, -z);
    near_vector(s.get_internal_vector(), z.get_internal_vector(), 2);
    mat4<T> m(z);
    Q recovered;
    recovered.from_euler_matrix(m);
    near_vector(recovered.get_internal_vector(), z.get_internal_vector(), 8);
    T angle;
    V axis;
    z.decomp_angle_axis(&angle, &axis);
    near_raw(angle, pi_v<T> / 2, 12);
    near_vector(axis, V(0, 0, 1));
    Q e;
    e.from_euler_angles(T::from_string("0.3"), T::from_string("-0.2"), T::from_string("0.1"));
    T yaw, pitch, roll;
    e.decomp_euler_angles(&yaw, &pitch, &roll);
    near_raw(yaw, T::from_string("0.3"), 12);
    near_raw(pitch, T::from_string("-0.2"), 12);
    near_raw(roll, T::from_string("0.1"), 12);
    near_vector(e.log().exp().get_internal_vector(), e.get_internal_vector(), 16);
    EXPECT_THROW(Q(T(1), V{}, true), std::domain_error);
}
TYPED_TEST(Geometry, LinesSegmentsCirclesAndPlanes) {
    using T = TypeParam;
    using V = vec2<T>;
    line2<T> line(T(1), T{}, T(-2));
    V out;
    line.project_point(V(7, 3), &out);
    EXPECT_EQ(out, V(2, 3));
    EXPECT_EQ(line.dist_to_point(V(5, 0)), T(3));
    line.normalize();
    line.a = 2;
    line.c = -4;
    EXPECT_EQ(line.dist_to_point(V(5, 0)), T(3));
    out = V(7, 3);
    line.project_point(out, &out);
    EXPECT_EQ(out, V(2, 3));
    segment2<T> segment(V(0, 0), V(4, 0));
    segment.get_closest_point(V(2, 3), &out);
    EXPECT_EQ(out, V(2, 0));
    EXPECT_EQ(segment.get_dist_to_point(V(2, 3)), T(3));
    EXPECT_EQ(segment.get_dist_to_point(V(7, 4)), T(5));
    segment2<T> point(V(1, 2), V(1, 2));
    point.get_closest_point(V(9, 2), &out);
    EXPECT_EQ(out, V(1, 2));
    EXPECT_TRUE(is_point_inside_triangle(V(0, 0), V(4, 0), V(0, 4), V(1, 1)));
    EXPECT_TRUE(is_point_inside_triangle(V(0, 0), V(4, 0), V(0, 4), V(2, 0)));
    EXPECT_TRUE(is_point_inside_triangle(V(0, 0), V(4, 0), V(2, 0), V(3, 0)));
    EXPECT_FALSE(is_point_inside_triangle(V(0, 0), V(4, 0), V(2, 0), V(5, 0)));
    circle<T> c(V{}, T(1)), c1, c2;
    EXPECT_FALSE(c.is_intersected(circle<T>(V(2, 0), T(1))));
    get_circles_by_tangent(V(1, 0), V{}, T(2), &c1, &c2);
    EXPECT_EQ(c1.center, V(0, 2));
    EXPECT_EQ(c2.center, V(0, -2));
    V p1, p2;
    EXPECT_FALSE(find_tangent_points(V{}, c, &p1, &p2));
    ASSERT_TRUE(find_tangent_points(V(1, 0), c, &p1, &p2));
    EXPECT_EQ(p1, V(1, 0));
    ASSERT_TRUE(find_tangent_points(V(5, 0), c, &p1, &p2));
    near_raw(length(p1), T(1), 8);
    near_raw(p1 * (p1 - V(5, 0)), T{}, 10);
    plane<T> plane;
    ASSERT_TRUE(plane.set(vec3<T>(0, 0, 2), vec3<T>(1, 0, 2), vec3<T>(0, 1, 2), true));
    EXPECT_EQ(plane.get_distance_to_point(vec3<T>(0, 0, 5)), T(3));
    EXPECT_EQ(plane.check_point_under_plane(vec3<T>{}), UINT32_C(0x80000000));
    EXPECT_FALSE(plane.set(vec3<T>{}, vec3<T>{}, vec3<T>{}, true));
    line2<T> invalid;
    EXPECT_THROW(invalid.normalize(), std::domain_error);
}
TYPED_TEST(Geometry, RectanglesRaysBoundsAndDirections) {
    using T = TypeParam;
    using V = vec2<T>;
    aabb2<T> box(T(-2), T(-1), T(2), T(1));
    EXPECT_EQ(box.width(), T(4));
    EXPECT_TRUE(box.is_inside(V(2, 1)));
    aabb2<T> adjacent(T(2), T(-1), T(3), T(1));
    EXPECT_FALSE(box.is_intersect(adjacent));
    EXPECT_TRUE(box.is_intersect_edges(adjacent));
    box.move_to(T{}, T{});
    EXPECT_EQ(box.get_center(), V(2, 1));
    box.inflate(T(1), T(2));
    box.deflate(T(1), T(2));
    EXPECT_EQ(box.width(), T(4));
    oriented_rect<T> rect;
    rect.init_rect(V{}, V(1, 0), T(2), T(1));
    EXPECT_TRUE(rect.is_point_inside(V{}));
    EXPECT_FALSE(rect.is_point_inside(V(2, 0)));
    oriented_rect<T> corners;
    corners.init_rect(rect.corner0(), rect.corner1(), rect.corner2(), rect.corner3());
    EXPECT_EQ(corners.length_ahead, T(2));
    EXPECT_EQ(corners.width, T(1));
    EXPECT_TRUE(rect.is_intersected(segment2<T>(V(-3, 0), V(3, 0))));
    EXPECT_FALSE(rect.is_intersected(segment2<T>(V(-4, 0), V(-3, 0))));
    EXPECT_TRUE(rect.is_intersected(segment2<T>(V{}, V{})));
    EXPECT_FALSE(rect.is_intersected(segment2<T>(V(3, 0), V(3, 0))));
    EXPECT_TRUE(rect.is_intersect_triangle(V{}, V(1, 0), V(0, T::from_string("0.5"))));
    EXPECT_TRUE(rect.is_intersect_circle(V(3, 0), T(1)));
    EXPECT_FALSE(rect.is_intersect_circle(V(4, 0), T(1)));
    V hit;
    ASSERT_TRUE(intersect_ray_rect(&hit, V(-3, 0), V(1, 0), rect));
    EXPECT_EQ(hit, V(-2, 0));
    EXPECT_FALSE(intersect_ray_rect(&hit, V(-3, 0), V(-1, 0), rect));
    ASSERT_TRUE(intersect_ray_rect(&hit, V{}, V(1, 0), rect));
    EXPECT_EQ(hit, V(2, 0));
    ASSERT_TRUE(intersect_ray_rect(&hit, V(-3, 1), V(1, 0), rect));
    EXPECT_EQ(hit, V(-2, 1));
    EXPECT_FALSE(intersect_ray_rect(&hit, V{}, V{}, rect));
    EXPECT_EQ(direction_by_vector(V(0, 1)), 0);
    EXPECT_EQ(direction_by_vector(V(-1, 0)), 16384);
    EXPECT_EQ(direction_by_vector(V(0, -1)), 32768);
    EXPECT_EQ(direction_by_vector(V(1, 0)), 49152);
    EXPECT_EQ(vector_by_direction<T>(0), V(0, 1));
    EXPECT_EQ(vector_by_direction<T>(49152), V(1, 0));
    EXPECT_EQ(direction_difference(65535, 0), 1);
    EXPECT_TRUE(is_in_the_angle(0, 65530, 10));
    EXPECT_FALSE(is_in_the_angle(100, 65530, 10));
    EXPECT_EQ(direction_by_vector(V(T::min(), T::min())), 24576);
    EXPECT_EQ(z_angle(T(1), T{}, T{}), 0);
    EXPECT_EQ(z_angle(T{}, T{}, T(1)), 16384);
    bound3<T> bound;
    bound.box_init(vec3<T>(-1, -2, -3), vec3<T>(1, 2, 3));
    EXPECT_TRUE(bound.is_inside(vec3<T>{}));
    EXPECT_TRUE(does_intersect(bound, bound));
    EXPECT_EQ(bound.half_box, vec3<T>(1, 2, 3));
    EXPECT_EQ(ray3<T>(vec3<T>{}, vec3<T>(1, 2, 3)).get(T(2)), vec3<T>(2, 4, 6));
}
TYPED_TEST(Geometry, ViewProjectionAndPacking) {
    using T = TypeParam;
    using V = vec3<T>;
    mat4<T> view, projection;
    create_view_matrix_lh(&view, V(1, 2, 3), V(1, 2, 4), V(0, 1, 0));
    V p(1, 2, 3);
    view.rotate_h_vector(&p, p);
    EXPECT_EQ(p, V{});
    create_perspective_projection_matrix_lh(&projection, pi_v<T> / 2, T(1), T(1), T(9));
    vec4<T> v;
    projection.rotate_h_vector(&v, V(0, 0, 1));
    EXPECT_EQ(v.z, T{});
    EXPECT_EQ(v.w, T(1));
    projection.rotate_h_vector(&v, V(0, 0, 9));
    near_raw(v.z / v.w, T(1), 2);
    create_perspective_projection_matrix_rh(&projection, pi_v<T> / 2, T(1), T(1), T(9));
    projection.rotate_h_vector(&v, V(0, 0, -1));
    EXPECT_EQ(v.z, T{});
    EXPECT_EQ(v.w, T(1));
    create_orthographic_projection_matrix_lh(&projection, T(4), T(2), T(1), T(9));
    projection.rotate_h_vector(&v, V(2, 1, 9));
    EXPECT_EQ(v, vec4<T>(1, 1, 1, 1));
    create_orthographic_projection_matrix_rh(&projection, T(4), T(2), T(1), T(9));
    projection.rotate_h_vector(&v, V(2, 1, -9));
    EXPECT_EQ(v, vec4<T>(1, 1, 1, 1));
    create_direct_transform_matrix(&projection, T(4), T(2));
    EXPECT_EQ(projection(0, 0), T::from_string("0.5"));
    EXPECT_THROW(create_view_matrix_lh(&view, V{}, V{}, V(0, 1, 0)), std::domain_error);
    EXPECT_EQ(norm_to_byte(T::from_string("0.5")), 128);
    EXPECT_EQ(norm_to_byte(T(2)), 255);
    EXPECT_EQ(fixed_to_byte(T(-1)), 129);
    EXPECT_EQ(byte_to_fixed<T>(129), T(-1));
    EXPECT_EQ(vec3_to_dword(V(-1, 0, 1)), UINT32_C(0x007f0081));
    EXPECT_EQ(dword_to_vec3<T>(UINT32_C(0x007f0081)), V(-1, 0, 1));
    EXPECT_EQ(get_binary32_bits(T(1)), UINT32_C(0x3f800000));
    EXPECT_EQ(reciprocal(T(4)), T::from_string("0.25"));
    EXPECT_EQ(exp_negative(T{}), T(1));
}
TEST(GeometryInteger, RasterizationAndGridArithmetic) {
    std::vector<int_vec2> points;
    auto emit = [&](std::int32_t x, std::int32_t y) { points.emplace_back(x, y); };
    bresenham_circle(3, 4, 0, emit);
    ASSERT_EQ(points.size(), 1U);
    EXPECT_EQ(points[0], int_vec2(3, 4));
    points.clear();
    bresenham_circle(0, 0, 2, emit);
    EXPECT_EQ(points.size(), 20U);
    EXPECT_NE(std::find(points.begin(), points.end(), int_vec2(2, 0)), points.end());
    points.clear();
    bresenham_filled_circle(0, 0, 2, emit);
    EXPECT_NE(std::find(points.begin(), points.end(), int_vec2(0, 0)), points.end());
    EXPECT_THROW(bresenham_circle(0, 0, -1, emit), std::domain_error);
    EXPECT_THROW(bresenham_circle(INT32_MAX, 0, 1, emit), std::overflow_error);
    std::vector<real2> ellipse;
    bresenham_ellipse(0, 0, 2, real(2), [&](std::int32_t x, real y) { ellipse.emplace_back(x, y); });
    EXPECT_NE(std::find(ellipse.begin(), ellipse.end(), real2(0, 4)), ellipse.end());
    int_vec2 v(3, 4);
    v.turn_left();
    EXPECT_EQ(v, int_vec2(-4, 3));
    v.turn_right();
    v /= 2;
    EXPECT_EQ(v, int_vec2(1, 2));
    EXPECT_EQ(grid_distance(int_vec2(INT32_MIN, 0), int_vec2(INT32_MAX, 0)), INT32_MAX);
    EXPECT_EQ(unpack_high_word(pack_dword(123, 456)), 123);
    EXPECT_EQ(unpack_byte3(pack_dword(1, 2, 3, 4)), 1);
}

TYPED_TEST(Geometry, TransformRoundTripsAndRemainingHelpers) {
    using T = TypeParam;
    using V = vec3<T>;
    using M = mat4<T>;
    using Q = quat<T>;
    std::uint64_t state = UINT64_C(0x123456789abcdef);
    for (std::size_t i = 0; i < 64; ++i) {
        auto random = [&] {
            return T::from_raw((static_cast<std::int64_t>(geometry_reference::next(state) % 2049) - 1024) *
                               static_cast<std::int64_t>(T::scale / 1024));
        };
        const auto x = random(), y = random(), z = random();
        Q q;
        q.from_euler_angles(x, y, z);
        const V v(x * 3, y * 2, z * 4);
        M m(V(1, 2, 3), q), inverse, unit;
        identity(&unit);
        ASSERT_TRUE(invert(&inverse, m));
        const auto product = m * inverse;
        for (std::size_t row = 0; row < 4; ++row)
            for (std::size_t col = 0; col < 4; ++col)
                near_raw(product(row, col), unit(row, col), 64);
        V result;
        m.rotate_vector(&result, v);
        near_vector(result, q.rotate(v), 16);
        m.rotate_h_vector(&result, v);
        inverse.rotate_h_vector(&result, result);
        near_vector(result, v, 96);
    }
    M m;
    Q q;
    make_matrix(&m, V(1, 2, 3), q, V(2, 3, 4));
    EXPECT_EQ(m(2, 2), T(4));
    make_matrix(&m, V{}, q);
    EXPECT_EQ(m(0, 0), T(1));
    create_view_matrix_lh(&m, V{}, Q{});
    create_view_matrix_rh(&m, V{}, Q{});
    M rotation;
    identity(&rotation);
    create_view_matrix_lh(&m, V{}, rotation);
    create_view_matrix_rh(&m, V{}, rotation);
    create_view_matrix_rh(&m, V{}, V(0, 0, 1), V(0, 1, 0));
    EXPECT_EQ(m(2, 2), T(-1));
    T phi, theta;
    get_angles(V{}, &phi, &theta);
    EXPECT_EQ(phi, T{});
    EXPECT_EQ(theta, T{});
    make_orientation(&q, V(0, 0, 1));
    EXPECT_EQ(q.get_internal_vector(), vec4<T>(0, 0, 0, 1));
    oriented_rect<T> a, b;
    a.init_rect(vec2<T>{}, vec2<T>(1, 0), T(1), T(1));
    b.init_rect(vec2<T>(5, 0), vec2<T>(1, 0), T(1), T(1));
    EXPECT_EQ(projected_distance(a, b), T(3));
    EXPECT_FALSE(a.is_intersected(b));
    EXPECT_GT(visible_angle(vec2<T>(5, 0), a), 0);
    EXPECT_EQ(a.get_side(vec2<T>(5, 0)), front);
    a.compress(T(2));
    EXPECT_EQ(a.width, T(2));
    polar<T> p(T(1), T(2));
    p += polar<T>(T(3), T(4));
    EXPECT_EQ(p.latitude, T(4));
    EXPECT_EQ(p.longitude, T(6));
    int_vec2 grid(3, 4);
    near_raw(length(grid.template norm<T>()), T(1), 2);
    EXPECT_EQ(grid.template to_vec2<T>(), vec2<T>(3, 4));
    EXPECT_EQ(z_direction(vec2<T>(1, 0), T{}), 49152);
    EXPECT_EQ(z_angle(V(0, 0, -1)), 49152);
    EXPECT_TRUE(is_in_the_min_angle(0, 65530, 10));
    EXPECT_EQ(direction_difference_sign(0, 1), -1);
}

TEST(GeometryReference, ExactCrossPlatformResults) {
    std::ifstream file(std::string(FXP_REFERENCE_DIR) + "/geometry.csv");
    ASSERT_TRUE(file);
    std::vector<std::string> expected;
    std::string line;
    while (std::getline(file, line))
        if (!line.empty() && line[0] != '#')
            expected.push_back(line);
    const auto actual = geometry_reference::generate();
    ASSERT_EQ(actual.size(), expected.size());
    ASSERT_GT(actual.size(), 5000U);
    for (std::size_t i = 0; i < actual.size(); ++i)
        EXPECT_EQ(actual[i], expected[i]) << "case " << i;
}

// Instantiate every non-template class method in both formats, including less
// common helpers that are not exercised by an individual numeric assertion.
#define FXP_INSTANTIATE_GEOMETRY(T)                                                                          \
    template class fxp::line2<T>;                                                                            \
    template class fxp::segment2<T>;                                                                         \
    template class fxp::circle<T>;                                                                           \
    template struct fxp::plane<T>;                                                                           \
    template struct fxp::mat4<T>;                                                                            \
    template class fxp::quat<T>;                                                                             \
    template class fxp::aabb2<T>;                                                                            \
    template class fxp::ray3<T>;                                                                             \
    template struct fxp::sphere<T>;                                                                          \
    template struct fxp::mass_sphere<T>;                                                                     \
    template struct fxp::bound3<T>;                                                                          \
    template struct fxp::oriented_rect<T>;
FXP_INSTANTIATE_GEOMETRY(fxp::real)
FXP_INSTANTIATE_GEOMETRY(fxp::fine)
#undef FXP_INSTANTIATE_GEOMETRY
