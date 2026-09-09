#pragma once
#include "vector.hpp"
#include <array>
#include <algorithm>
#include <cstdint>

namespace fxp {
template <class T = real> class quat;
template <class T> struct oriented_rect;
template <class T> bool is_almost_zero(T, T);
template <class T> T triangle_area2(const vec2<T> &, const vec2<T> &, const vec2<T> &);
template <class T = real> class line2 {
    static_assert(detail::is_fixed_v<T>, "geometry requires real or fine");

  public:
    T a{}, b{}, c{};

    line2() {}
    line2(const T _a, const T _b, const T _c) : a(_a), b(_b), c(_c) {}
    line2(const vec2<T> &p1, const vec2<T> &p2)
        : a(p2.y - p1.y), b(p1.x - p2.x), c(p2.x * p1.y - p1.x * p2.y) {}

    T dist_to_point(const vec2<T> &point);

    void project_point(const vec2<T> &point, vec2<T> *result);

    int get_sign(const vec2<T> &point) const { return geometry_detail::sign(a * point.x + b * point.y + c); }

    void normalize() {
        if (a == 0 && b == 0)
            throw std::domain_error("zero line normal");
        const T divisor = length(a, b);
        a /= divisor;
        b /= divisor;
        c /= divisor;
    }
};

template <class T = real> class segment2 {
  public:
    vec2<T> p1, p2;
    vec2<T> direction;

    segment2() {}
    segment2(const vec2<T> &_p1, const vec2<T> &_p2) : p1(_p1), p2(_p2), direction(p2 - p1) {}
    segment2(const T x1, const T y1, const T x2, const T y2) : p1(x1, y1), p2(x2, y2), direction(p2 - p1) {}

    T get_dist_to_point(const vec2<T> &point) const;

    void get_closest_point(const vec2<T> &point, vec2<T> *result) const;
};

template <class T = real> class circle {
  public:
    vec2<T> center;
    T radius{};

    circle() {}
    circle(const vec2<T> &_center, const T _r) : center(_center), radius(_r) {}

    const vec2<T> &get_center() const { return center; }
    bool is_intersected(const circle<T> &circle) const {
        const T fDist = length(circle.center - center);
        return fDist < radius + circle.radius;
    }
};

template <class T>
inline void get_circles_by_tangent(const vec2<T> &tang, const vec2<T> &p, const T r, circle<T> *c1,
                                   circle<T> *c2);

template <class T>
inline bool find_tangent_points(const vec2<T> &p, const circle<T> &c, vec2<T> *p1, vec2<T> *p2);

template <class T> inline T triangle_area2(const vec2<T> &p1, const vec2<T> &p2, const vec2<T> &p3);

template <class T>
inline bool is_point_inside_triangle(const vec2<T> &p1, const vec2<T> &p2, const vec2<T> &p3,
                                     const vec2<T> &p);

template <class T = real> struct plane {
  public:
    vec3<T> n;
    T d{};
    vec4<T> components() const noexcept { return vec4<T>(n.x, n.y, n.z, d); }
    plane() {}
    plane(const vec3<T> &vNormale, const T fDist) : n(vNormale), d(fDist) {}
    plane(const vec4<T> &v) : n(v.x, v.y, v.z), d(v.w) {}
    plane(const plane &) = default;
    plane &operator=(const plane &) = default;

    bool set(const vec3<T> &pt0, const vec3<T> &pt1, const vec3<T> &pt2, bool bNormalize);
    void set(const vec3<T> &pt0, const vec3<T> &pt1, const vec3<T> &pt2);
    void set(const vec3<T> &vNormale, const T fDist) {
        n = vNormale;
        d = fDist;
    }

    void recalc_dist(const vec3<T> &pt) { d = -(n * pt); }

    T get_distance_to_point(const vec3<T> &pt) const { return (n * pt + d); }
    bool is_point_on_plane(const vec3<T> &pt) const { return n * pt == -d; }
    bool is_point_over_plane(const vec3<T> &pt) const { return n * pt > -d; }
    bool is_point_under_plane(const vec3<T> &pt) const { return n * pt < -d; }

    std::uint32_t check_point_under_plane(const vec3<T> &pt) const;
};

template <class T = real> struct mat4 {
    static_assert(detail::is_fixed_v<T>, "geometry requires real or fine");
    std::array<std::array<T, 4>, 4> elements{};
    T &at(std::size_t row, std::size_t col) { return elements.at(row).at(col); }
    const T &at(std::size_t row, std::size_t col) const { return elements.at(row).at(col); }
    auto &operator[](std::size_t row) { return elements.at(row); }
    const auto &operator[](std::size_t row) const { return elements.at(row); }
    T &operator()(std::size_t row, std::size_t col) { return at(row, col); }
    const T &operator()(std::size_t row, std::size_t col) const { return at(row, col); }
    mat4() = default;
    template <class U> mat4(const mat4<U> &source) {
        for (std::size_t r = 0; r < 4; ++r)
            for (std::size_t c = 0; c < 4; ++c)
                at(r, c) = T(source.at(r, c));
    }
    mat4(T a00, T a01, T a02, T a03, T a10, T a11, T a12, T a13, T a20, T a21, T a22, T a23, T a30, T a31,
         T a32, T a33)
        : elements{{{{a00, a01, a02, a03}},
                    {{a10, a11, a12, a13}},
                    {{a20, a21, a22, a23}},
                    {{a30, a31, a32, a33}}}} {}
    mat4(const quat<T> &rotation) { set(rotation); }
    mat4(const vec3<T> &position, const quat<T> &rotation) { set(position, rotation); }
    void set(const quat<T> &rotation);
    void set(const vec3<T> &position, const quat<T> &rotation);
    void set(T, T, T, T, T, T, T, T, T, T, T, T, T, T, T, T);
    void set_x(const vec4<T> &v) { elements[0] = {v.x, v.y, v.z, v.w}; }
    void set_y(const vec4<T> &v) { elements[1] = {v.x, v.y, v.z, v.w}; }
    void set_z(const vec4<T> &v) { elements[2] = {v.x, v.y, v.z, v.w}; }
    void set_w(const vec4<T> &v) { elements[3] = {v.x, v.y, v.z, v.w}; }
    vec4<T> x() const { return vec4<T>(at(0, 0), at(0, 1), at(0, 2), at(0, 3)); }
    vec4<T> y() const { return vec4<T>(at(1, 0), at(1, 1), at(1, 2), at(1, 3)); }
    vec4<T> z() const { return vec4<T>(at(2, 0), at(2, 1), at(2, 2), at(2, 3)); }
    vec4<T> w() const { return vec4<T>(at(3, 0), at(3, 1), at(3, 2), at(3, 3)); }
    vec3<T> x3() const { return vec3<T>(at(0, 0), at(0, 1), at(0, 2)); }
    vec3<T> y3() const { return vec3<T>(at(1, 0), at(1, 1), at(1, 2)); }
    vec3<T> z3() const { return vec3<T>(at(2, 0), at(2, 1), at(2, 2)); }
    vec3<T> w3() const { return vec3<T>(at(3, 0), at(3, 1), at(3, 2)); }
    vec3<T> get_x_axis3() const { return x3(); }
    vec3<T> get_y_axis3() const { return y3(); }
    vec3<T> get_z_axis3() const { return z3(); }
    vec3<T> get_trans3() const { return vec3<T>(at(0, 3), at(1, 3), at(2, 3)); }
    vec4<T> get_trans4() const { return vec4<T>(at(0, 3), at(1, 3), at(2, 3), at(3, 3)); }
    vec3<T> get_translation() const { return get_trans3(); }
    void transform(vec3<T> *out, const vec3<T> &v) const { rotate_vector(out, v); }
    void transform_homogeneous(vec3<T> *out, const vec3<T> &v) const { rotate_h_vector(out, v); }
    void rotate_vector(vec3<T> *, const vec3<T> &) const;
    void rotate_vector_transposed(vec3<T> *, const vec3<T> &) const;
    void rotate_h_direction(vec4<T> *, const vec3<T> &) const;
    void rotate_h_vector(vec3<T> *, const vec3<T> &) const;
    void rotate_h_vector(vec4<T> *, const vec3<T> &) const;
    void rotate_h_vector(vec4<T> *, const vec4<T> &) const;
    void rotate_h_vector_transposed(vec4<T> *, const vec4<T> &) const;
    bool homogeneous_inverse(const mat4<T> &);
};
template <class T = real> inline bool operator==(const mat4<T> &a, const mat4<T> &b) {
    return a.elements == b.elements;
}
template <class T = real> inline bool operator!=(const mat4<T> &a, const mat4<T> &b) {
    return !(a == b);
}

template <class T = real> struct transform_pair {
    mat4<T> forward, backward;
};

template <class T> class quat {
    static_assert(detail::is_fixed_v<T>, "geometry requires real or fine");
    vec4<T> components_{0, 0, 0, 1};

    quat(T fX, T fY, T fZ, T fW, int, int) : components_(fX, fY, fZ, fW) {}

  public:
    quat(T fAngle, T fAxisX, T fAxisY, T fAxisZ, bool bNormalizeAxis = false);
    quat(T fAngle, const vec3<T> &vAxis, bool bNormalizeAxis = false);
    quat(const vec4<T> &quat) { components_ = quat; }
    quat() = default;
    template <class U> quat(const quat<U> &source) : components_(source.get_internal_vector()) {}

    void from_angle_axis(T fAngle, const vec3<T> &vAxis, bool bNormalizeAxis = false);
    void from_angle_axis(T fAngle, T fAxisX, T fAxisY, T fAxisZ, bool bNormalizeAxis = false);
    void from_euler_matrix(const mat4<T> &m);
    void from_euler_angles(T yaw, T pitch, T roll);
    void from_components(T _x, T _y, T _z, T _w) { components_ = vec4<T>{_x, _y, _z, _w}; }
    void from_components(const vec4<T> &_vec4) { components_ = _vec4; }

    void decomp_angle_axis(T *pfAngle, vec3<T> *pvAxis) const;
    void decomp_angle_axis(T *pfAngle, T *pfAxisX, T *pfAxisY, T *pfAxisZ) const;
    void decomp_euler_matrix(mat4<T> *pMatrix) const;
    void decomp_euler_angles(T *pfYaw, T *pfPitch, T *pfRoll);
    void decomp_reversed_euler_matrix(mat4<T> *pMatrix) const;

    bool normalize() { return fxp::normalize(components_.x, components_.y, components_.z, components_.w); }
    void maximize(const quat<T> &v) {
        components_.x = (std::max)(components_.x, v.components_.x);
        components_.y = (std::max)(components_.y, v.components_.y);
        components_.z = (std::max)(components_.z, v.components_.z);
        components_.w = (std::max)(components_.w, v.components_.w);
    }
    void minimize(const quat<T> &v) {
        components_.x = (std::min)(components_.x, v.components_.x);
        components_.y = (std::min)(components_.y, v.components_.y);
        components_.z = (std::min)(components_.z, v.components_.z);
        components_.w = (std::min)(components_.w, v.components_.w);
    }

    void negate(const quat<T> &q) {
        components_.x = -q.components_.x;
        components_.y = -q.components_.y;
        components_.z = -q.components_.z;
        components_.w = -q.components_.w;
    }
    void negate() {
        components_.x = -components_.x;
        components_.y = -components_.y;
        components_.z = -components_.z;
        components_.w = -components_.w;
    }
    bool inverse(const quat<T> &q);
    bool inverse();
    void unit_inverse(const quat<T> &q) {
        components_.x = -q.components_.x;
        components_.y = -q.components_.y;
        components_.z = -q.components_.z;
        components_.w = q.components_.w;
    }
    void unit_inverse() {
        components_.x = -components_.x;
        components_.y = -components_.y;
        components_.z = -components_.z;
    }
    void unit_inverse_x() { components_.x = -components_.x; }
    void unit_inverse_y() { components_.y = -components_.y; }
    void unit_inverse_z() { components_.z = -components_.z; }

    void deriv(const quat<T> &q, const vec3<T> &v);

    template <class U> friend const quat<U> operator*(const quat<U> &a, const quat<U> &b);
    template <class U> friend const quat<U> operator/(const quat<U> &a, const quat<U> &b);
    quat<T> &operator*=(const quat<T> &quat);
    quat<T> &operator/=(const quat<T> &quat);
    const quat<T> operator+(const quat<T> &q) const {
        return quat(components_.x + q.components_.x, components_.y + q.components_.y,
                    components_.z + q.components_.z, components_.w + q.components_.w, 0, 0);
    }
    const quat<T> operator-() const {
        return quat(-components_.x, -components_.y, -components_.z, -components_.w, 0, 0);
    }
    T dot(const quat<T> &quat) const {
        return components_.x * quat.components_.x + components_.y * quat.components_.y +
               components_.z * quat.components_.z + components_.w * quat.components_.w;
    }

    void minimize_rotation_angle() {
        if (components_.w < 0) {
            components_.x = -components_.x;
            components_.y = -components_.y;
            components_.z = -components_.z;
            components_.w = -components_.w;
        }
    }

    const quat<T> exp() const;
    const quat<T> log() const;

    void slerp(const T factor, const quat<T> &p, const quat<T> &q);
    void interpolate(const quat<T> &p, const quat<T> &q, const T t) { slerp(t, p, q); }

    const vec3<T> rotate(const vec3<T> &r) const;
    void rotate(vec3<T> *pRes, const vec3<T> &vec) const;

    const vec3<T> get_x_axis() const;
    const vec3<T> get_y_axis() const;
    const vec3<T> get_z_axis() const;
    void get_x_axis(vec3<T> *pResult) const;
    void get_y_axis(vec3<T> *pResult) const;
    void get_z_axis(vec3<T> *pResult) const;

    friend T length_squared(const quat<T> &q) {
        return length_squared(q.components_.x, q.components_.y, q.components_.z, q.components_.w);
    }
    friend T length(const quat<T> &q) { return static_cast<T>(sqrt(length_squared(q))); }

    const vec4<T> &get_internal_vector() const { return components_; }
};
template <class T = real> inline const quat<T> identity_quat_v = quat<T>(0, 1, 0, 0);

template <class T = real> class aabb2 {
    static_assert(detail::is_fixed_v<T>, "geometry requires real or fine");

  public:
    using TRect = aabb2<T>;
    using TPoint = point2<T>;
    T minx{}, miny{}, maxx{}, maxy{};

    aabb2() {}
    aabb2(const T _minx, const T _miny, const T _maxx, const T _maxy)
        : minx(_minx), miny(_miny), maxx(_maxx), maxy(_maxy) {}
    aabb2(const TPoint &vLT, const TPoint &vRB) : minx(vLT.x), miny(vLT.y), maxx(vRB.x), maxy(vRB.y) {}
    template <class T1>
    aabb2(const aabb2<T1> &rect) : minx(rect.minx), miny(rect.miny), maxx(rect.maxx), maxy(rect.maxy) {}
    aabb2(const TRect &rect) : minx(rect.minx), miny(rect.miny), maxx(rect.maxx), maxy(rect.maxy) {}

    template <class T1> const TRect &operator=(const aabb2<T1> &rect) {
        minx = rect.minx;
        miny = rect.miny;
        maxx = rect.maxx;
        maxy = rect.maxy;
        return *this;
    }
    const TRect &operator=(const TRect &rect) {
        minx = rect.minx;
        miny = rect.miny;
        maxx = rect.maxx;
        maxy = rect.maxy;
        return *this;
    }
    void set(const T _x1, const T _y1, const T _x2, const T _y2) {
        minx = _x1;
        miny = _y1;
        maxx = _x2;
        maxy = _y2;
    }
    void set(const TPoint &vLT, const TPoint &vRB) {
        minx = vLT.x;
        miny = vLT.y;
        maxx = vRB.x;
        maxy = vRB.y;
    }
    void set(const TRect &rect) {
        minx = rect.minx;
        miny = rect.miny;
        maxx = rect.maxx;
        maxy = rect.maxy;
    }
    void set_empty() { minx = miny = maxx = maxy = 0; }
    void set_rect(const T _x1, const T _y1, const T _x2, const T _y2) { set(_x1, _y1, _x2, _y2); }
    void set_rect(const TPoint &vLT, const TPoint &vRB) { set(vLT, vRB); }
    void set_rect(const TRect &rect) { set(rect); }
    void set_rect_empty() { set_empty(); }

    void operator*=(T fScale) {
        minx *= fScale;
        miny *= fScale;
        maxx *= fScale;
        maxy *= fScale;
    }

    T width() const { return (maxx - minx); }
    T height() const { return (maxy - miny); }
    T get_size_x() const { return width(); }
    T get_size_y() const { return height(); }
    const TPoint get_size() const { return TPoint(width(), height()); }
    T get_area() const { return width() * height(); }

    const TPoint get_left_top() const { return TPoint(minx, miny); }
    const TPoint get_right_top() const { return TPoint(maxx, miny); }
    const TPoint get_left_bottom() const { return TPoint(minx, maxy); }
    const TPoint get_right_bottom() const { return TPoint(maxx, maxy); }
    const TPoint get_center() const { return TPoint((minx + maxx) / T(2), (miny + maxy) / T(2)); }

    bool operator==(const TRect &rc) const {
        return (minx == rc.minx) && (miny == rc.miny) && (maxx == rc.maxx) && (maxy == rc.maxy);
    }
    bool operator!=(const TRect &rc) const {
        return (minx != rc.minx) || (miny != rc.miny) || (maxx != rc.maxx) || (maxy != rc.maxy);
    }
    bool is_empty() const { return ((minx >= maxx) || (miny >= maxy)); }
    bool is_inside(const TPoint &pt) const {
        return (pt.x >= minx) && (pt.x <= maxx) && (pt.y >= miny) && (pt.y <= maxy);
    }
    bool is_inside(const T &x, const T &y) const {
        return (x >= minx) && (x <= maxx) && (y >= miny) && (y <= maxy);
    }
    bool is_inside(const TRect &rect) const {
        return (rect.minx >= minx) && (rect.maxx <= maxx) && (rect.miny >= miny) && (rect.maxy <= maxy);
    }
    bool is_intersect(const TRect &rc) const {
        return ((std::max)(minx, rc.minx) < (std::min)(maxx, rc.maxx)) &&
               ((std::max)(miny, rc.miny) < (std::min)(maxy, rc.maxy));
    }
    bool is_intersect_edges(const TRect &rc) const {
        return ((std::max)(minx, rc.minx) <= (std::min)(maxx, rc.maxx)) &&
               ((std::max)(miny, rc.miny) <= (std::min)(maxy, rc.maxy));
    }

    void intersect(const TRect &rect) {
        minx = (std::max)(minx, rect.minx);
        maxx = (std::min)(maxx, rect.maxx);
        miny = (std::max)(miny, rect.miny);
        maxy = (std::min)(maxy, rect.maxy);
    }
    TRect &unite(const TRect &rect) {
        if (is_empty())
            *this = rect;
        else if (!rect.is_empty()) {
            minx = (std::min)(minx, rect.minx);
            maxx = (std::max)(maxx, rect.maxx);
            miny = (std::min)(miny, rect.miny);
            maxy = (std::max)(maxy, rect.maxy);
        }
        return *this;
    }

    void inflate(const T halfX, const T halfY) {
        minx -= halfX;
        miny -= halfY;
        maxx += halfX;
        maxy += halfY;
    }
    void deflate(const T halfX, const T halfY) {
        minx += halfX;
        miny += halfY;
        maxx -= halfX;
        maxy -= halfY;
    }

    void move_to(const T x, const T y) {
        maxx += x - minx;
        maxy += y - miny;
        minx = x;
        miny = y;
    }
    void move_to(const TPoint &pt) { move_to(pt.x, pt.y); }

    void move(const T dx, const T dy) {
        minx += dx;
        miny += dy;
        maxx += dx;
        maxy += dy;
    }
    void move(const TPoint &pt) { move(pt.x, pt.y); }

    void normalize() {
        set((std::min)(minx, maxx), (std::min)(miny, maxy), (std::max)(minx, maxx), (std::max)(miny, maxy));
    }
};

struct triangle_indices {
    std::uint16_t i1{}, i2{}, i3{};

    triangle_indices() {}
    triangle_indices(std::uint16_t _i1, std::uint16_t _i2, std::uint16_t _i3) : i1(_i1), i2(_i2), i3(_i3) {}

    void set(std::uint16_t _i1, std::uint16_t _i2, std::uint16_t _i3) {
        i1 = _i1;
        i2 = _i2;
        i3 = _i3;
    }
};

template <class T = real> class ray3 {
  public:
    vec3<T> origin, direction;

    ray3() {}
    ray3(const vec3<T> &_ptOrigin, const vec3<T> &_ptDir) : origin(_ptOrigin), direction(_ptDir) {}
    vec3<T> get(T fT) const { return origin + direction * fT; }
};

template <class T = real> struct sphere {
    vec3<T> center;
    T radius{};

    sphere() {}
    sphere(const vec3<T> &_ptCenter, T _fRadius) : center(_ptCenter), radius(_fRadius) {}
};

template <class T = real> struct mass_sphere {
    vec3<T> center;
    T radius{};
    T mass{};

    mass_sphere() {}
    mass_sphere(const vec3<T> &_ptCenter, T _fRadius, T _fMass)
        : center(_ptCenter), radius(_fRadius), mass(_fMass) {}
};

template <class T = real> struct bound3 {
    sphere<T> s;
    vec3<T> half_box;

    void box_init(const vec3<T> &ptMin, const vec3<T> &ptMax) {
        half_box = (ptMax - ptMin) * T::from_string("0.5");
        s.center = (ptMax + ptMin) * T::from_string("0.5");
        s.radius = length(half_box);
    }
    void box_ex_init(const vec3<T> &_ptCenter, const vec3<T> &_ptHalfBox) {
        s.center = _ptCenter;
        s.radius = length(_ptHalfBox);
        half_box = _ptHalfBox;
    }
    void sphere_init(const vec3<T> &_ptCenter, T radius) {
        s.center = _ptCenter;
        s.radius = radius;
        half_box = vec3<T>(radius, radius, radius);
    }
    void extend(T fAxisHalfSize) {
        half_box.x += fAxisHalfSize;
        half_box.y += fAxisHalfSize;
        half_box.z += fAxisHalfSize;
        s.radius = length(half_box);
    }
    bool is_inside(const vec3<T> &v) {
        vec3<T> vTest(s.center - v);
        if (length_squared(vTest) > length_squared(s.radius))
            return false;
        return length(vTest.x) <= half_box.x && length(vTest.y) < half_box.y && length(vTest.z) < half_box.z;
    }
};

template <class T> inline bool does_intersect(const bound3<T> &a, const bound3<T> &b) {
    vec3<T> ptDif = a.s.center - b.s.center;
    if (length_squared(ptDif) > length_squared(a.s.radius + b.s.radius))
        return false;
    if (length(ptDif.x) > a.half_box.x + b.half_box.x || length(ptDif.y) > a.half_box.y + b.half_box.y ||
        length(ptDif.z) > a.half_box.z + b.half_box.z)
        return false;
    return true;
}

template <class T>
inline bool plane<T>::set(const vec3<T> &pt0, const vec3<T> &pt1, const vec3<T> &pt2, bool bNormalize) {
    vec3<T> v1(pt1.x - pt0.x, pt1.y - pt0.y, pt1.z - pt0.z), v2(pt2.x - pt0.x, pt2.y - pt0.y, pt2.z - pt0.z);
    if (bNormalize && (!normalize(&v1) || !normalize(&v2))) {
        return false;
    }

    n = v1 ^ v2;
    if (bNormalize && !normalize(&n)) {
        return false;
    }

    d = -(pt0 * n);

    return true;
}
template <class T> inline void plane<T>::set(const vec3<T> &pt0, const vec3<T> &pt1, const vec3<T> &pt2) {
    vec3<T> v1(pt1.x - pt0.x, pt1.y - pt0.y, pt1.z - pt0.z), v2(pt2.x - pt0.x, pt2.y - pt0.y, pt2.z - pt0.z);

    n = v1 ^ v2;

    d = -(pt0 * n);
}

template <class T> inline std::uint32_t plane<T>::check_point_under_plane(const vec3<T> &pt) const {
    T fDist = n * pt + d;
    return (fDist < 0 ? UINT32_C(0x80000000) : UINT32_C(0));
}

template <class T> inline void identity(mat4<T> *pRes) {
    *pRes = mat4<T>{};

    pRes->at(0, 0) = pRes->at(1, 1) = pRes->at(2, 2) = pRes->at(3, 3) = T(1);
}

template <class T> inline void transpose(mat4<T> *p, const mat4<T> &m) {
    p->set(m.at(0, 0), m.at(1, 0), m.at(2, 0), m.at(3, 0), m.at(0, 1), m.at(1, 1), m.at(2, 1), m.at(3, 1),
           m.at(0, 2), m.at(1, 2), m.at(2, 2), m.at(3, 2), m.at(0, 3), m.at(1, 3), m.at(2, 3), m.at(3, 3));
}

template <class T> inline void mat4<T>::set(const quat<T> &quat) {

    quat.decomp_euler_matrix(this);

    at(0, 3) = at(1, 3) = at(2, 3) = at(3, 0) = at(3, 1) = at(3, 2) = 0;
    at(3, 3) = 1;
}
template <class T> inline void mat4<T>::set(const vec3<T> &vPos, const quat<T> &quat) {

    quat.decomp_euler_matrix(this);

    at(0, 3) = vPos.x;
    at(1, 3) = vPos.y;
    at(2, 3) = vPos.z;

    at(3, 0) = at(3, 1) = at(3, 2) = 0;
    at(3, 3) = 1;
}
template <class T>
inline void mat4<T>::set(T __11, T __12, T __13, T __14, T __21, T __22, T __23, T __24, T __31, T __32,
                         T __33, T __34, T __41, T __42, T __43, T __44) {
    at(0, 0) = __11;
    at(0, 1) = __12;
    at(0, 2) = __13;
    at(0, 3) = __14;
    at(1, 0) = __21;
    at(1, 1) = __22;
    at(1, 2) = __23;
    at(1, 3) = __24;
    at(2, 0) = __31;
    at(2, 1) = __32;
    at(2, 2) = __33;
    at(2, 3) = __34;
    at(3, 0) = __41;
    at(3, 1) = __42;
    at(3, 2) = __43;
    at(3, 3) = __44;
}

template <class T> inline void multiply(mat4<T> *destination, const mat4<T> &a, const mat4<T> &b) {
    geometry_detail::output(destination);
    mat4<T> result;
    auto *p = &result;
    p->at(0, 0) =
        a.at(0, 0) * b.at(0, 0) + a.at(0, 1) * b.at(1, 0) + a.at(0, 2) * b.at(2, 0) + a.at(0, 3) * b.at(3, 0);
    p->at(0, 1) =
        a.at(0, 0) * b.at(0, 1) + a.at(0, 1) * b.at(1, 1) + a.at(0, 2) * b.at(2, 1) + a.at(0, 3) * b.at(3, 1);
    p->at(0, 2) =
        a.at(0, 0) * b.at(0, 2) + a.at(0, 1) * b.at(1, 2) + a.at(0, 2) * b.at(2, 2) + a.at(0, 3) * b.at(3, 2);
    p->at(0, 3) =
        a.at(0, 0) * b.at(0, 3) + a.at(0, 1) * b.at(1, 3) + a.at(0, 2) * b.at(2, 3) + a.at(0, 3) * b.at(3, 3);

    p->at(1, 0) =
        a.at(1, 0) * b.at(0, 0) + a.at(1, 1) * b.at(1, 0) + a.at(1, 2) * b.at(2, 0) + a.at(1, 3) * b.at(3, 0);
    p->at(1, 1) =
        a.at(1, 0) * b.at(0, 1) + a.at(1, 1) * b.at(1, 1) + a.at(1, 2) * b.at(2, 1) + a.at(1, 3) * b.at(3, 1);
    p->at(1, 2) =
        a.at(1, 0) * b.at(0, 2) + a.at(1, 1) * b.at(1, 2) + a.at(1, 2) * b.at(2, 2) + a.at(1, 3) * b.at(3, 2);
    p->at(1, 3) =
        a.at(1, 0) * b.at(0, 3) + a.at(1, 1) * b.at(1, 3) + a.at(1, 2) * b.at(2, 3) + a.at(1, 3) * b.at(3, 3);

    p->at(2, 0) =
        a.at(2, 0) * b.at(0, 0) + a.at(2, 1) * b.at(1, 0) + a.at(2, 2) * b.at(2, 0) + a.at(2, 3) * b.at(3, 0);
    p->at(2, 1) =
        a.at(2, 0) * b.at(0, 1) + a.at(2, 1) * b.at(1, 1) + a.at(2, 2) * b.at(2, 1) + a.at(2, 3) * b.at(3, 1);
    p->at(2, 2) =
        a.at(2, 0) * b.at(0, 2) + a.at(2, 1) * b.at(1, 2) + a.at(2, 2) * b.at(2, 2) + a.at(2, 3) * b.at(3, 2);
    p->at(2, 3) =
        a.at(2, 0) * b.at(0, 3) + a.at(2, 1) * b.at(1, 3) + a.at(2, 2) * b.at(2, 3) + a.at(2, 3) * b.at(3, 3);

    p->at(3, 0) =
        a.at(3, 0) * b.at(0, 0) + a.at(3, 1) * b.at(1, 0) + a.at(3, 2) * b.at(2, 0) + a.at(3, 3) * b.at(3, 0);
    p->at(3, 1) =
        a.at(3, 0) * b.at(0, 1) + a.at(3, 1) * b.at(1, 1) + a.at(3, 2) * b.at(2, 1) + a.at(3, 3) * b.at(3, 1);
    p->at(3, 2) =
        a.at(3, 0) * b.at(0, 2) + a.at(3, 1) * b.at(1, 2) + a.at(3, 2) * b.at(2, 2) + a.at(3, 3) * b.at(3, 2);
    p->at(3, 3) =
        a.at(3, 0) * b.at(0, 3) + a.at(3, 1) * b.at(1, 3) + a.at(3, 2) * b.at(2, 3) + a.at(3, 3) * b.at(3, 3);
    *destination = result;
}
template <class T>
inline void multiply_scale(mat4<T> *destination, const mat4<T> &a, const T fX, const T fY, const T fZ) {
    geometry_detail::output(destination);
    mat4<T> result;
    auto *p = &result;
    p->at(0, 0) = a.at(0, 0) * fX;
    p->at(0, 1) = a.at(0, 1) * fY;
    p->at(0, 2) = a.at(0, 2) * fZ;
    p->at(0, 3) = a.at(0, 3);

    p->at(1, 0) = a.at(1, 0) * fX;
    p->at(1, 1) = a.at(1, 1) * fY;
    p->at(1, 2) = a.at(1, 2) * fZ;
    p->at(1, 3) = a.at(1, 3);

    p->at(2, 0) = a.at(2, 0) * fX;
    p->at(2, 1) = a.at(2, 1) * fY;
    p->at(2, 2) = a.at(2, 2) * fZ;
    p->at(2, 3) = a.at(2, 3);

    p->at(3, 0) = a.at(3, 0) * fX;
    p->at(3, 1) = a.at(3, 1) * fY;
    p->at(3, 2) = a.at(3, 2) * fZ;
    p->at(3, 3) = a.at(3, 3);
    *destination = result;
}

template <class T> inline const mat4<T> operator*(const mat4<T> &a, const mat4<T> &b) {
    mat4<T> ret;
    multiply(&ret, a, b);
    return ret;
}
template <class T = real>
inline transform_pair<T> operator*(const transform_pair<T> &a, const transform_pair<T> &b) {
    transform_pair<T> res;
    res.forward = a.forward * b.forward;
    res.backward = b.backward * a.backward;
    return res;
}

template <class T> inline void mat4<T>::rotate_vector(vec3<T> *pResult, const vec3<T> &pt) const {
    const T x = at(0, 0) * pt.x + at(0, 1) * pt.y + at(0, 2) * pt.z;
    const T y = at(1, 0) * pt.x + at(1, 1) * pt.y + at(1, 2) * pt.z;
    const T z = at(2, 0) * pt.x + at(2, 1) * pt.y + at(2, 2) * pt.z;
    pResult->set(x, y, z);
}
template <class T> inline void mat4<T>::rotate_vector_transposed(vec3<T> *pResult, const vec3<T> &pt) const {
    const T x = pt.x, y = pt.y, z = pt.z;
    pResult->x = at(0, 0) * x + at(1, 0) * y + at(2, 0) * z;
    pResult->y = at(0, 1) * x + at(1, 1) * y + at(2, 1) * z;
    pResult->z = at(0, 2) * x + at(1, 2) * y + at(2, 2) * z;
}
template <class T> inline void mat4<T>::rotate_h_direction(vec4<T> *pResult, const vec3<T> &pt) const {
    const T x = pt.x, y = pt.y, z = pt.z;
    pResult->x = at(0, 0) * x + at(0, 1) * y + at(0, 2) * z;
    pResult->y = at(1, 0) * x + at(1, 1) * y + at(1, 2) * z;
    pResult->z = at(2, 0) * x + at(2, 1) * y + at(2, 2) * z;
    pResult->w = at(3, 0) * x + at(3, 1) * y + at(3, 2) * z;
}
template <class T> inline void mat4<T>::rotate_h_vector(vec3<T> *pResult, const vec3<T> &pt) const {
    const T x = at(0, 0) * pt.x + at(0, 1) * pt.y + at(0, 2) * pt.z + at(0, 3);
    const T y = at(1, 0) * pt.x + at(1, 1) * pt.y + at(1, 2) * pt.z + at(1, 3);
    const T z = at(2, 0) * pt.x + at(2, 1) * pt.y + at(2, 2) * pt.z + at(2, 3);
    pResult->set(x, y, z);
}
template <class T> inline void mat4<T>::rotate_h_vector(vec4<T> *pResult, const vec3<T> &pt) const {
    const T x = at(0, 0) * pt.x + at(0, 1) * pt.y + at(0, 2) * pt.z + at(0, 3);
    const T y = at(1, 0) * pt.x + at(1, 1) * pt.y + at(1, 2) * pt.z + at(1, 3);
    const T z = at(2, 0) * pt.x + at(2, 1) * pt.y + at(2, 2) * pt.z + at(2, 3);
    const T w = at(3, 0) * pt.x + at(3, 1) * pt.y + at(3, 2) * pt.z + at(3, 3);
    pResult->set(x, y, z, w);
}
template <class T> inline void mat4<T>::rotate_h_vector(vec4<T> *pResult, const vec4<T> &pt) const {
    const T x = at(0, 0) * pt.x + at(0, 1) * pt.y + at(0, 2) * pt.z + at(0, 3) * pt.w;
    const T y = at(1, 0) * pt.x + at(1, 1) * pt.y + at(1, 2) * pt.z + at(1, 3) * pt.w;
    const T z = at(2, 0) * pt.x + at(2, 1) * pt.y + at(2, 2) * pt.z + at(2, 3) * pt.w;
    const T w = at(3, 0) * pt.x + at(3, 1) * pt.y + at(3, 2) * pt.z + at(3, 3) * pt.w;
    pResult->set(x, y, z, w);
}

template <class T>
inline void mat4<T>::rotate_h_vector_transposed(vec4<T> *pResult, const vec4<T> &pt) const {
    const T x = at(0, 0) * pt.x + at(1, 0) * pt.y + at(2, 0) * pt.z + at(3, 0) * pt.w;
    const T y = at(0, 1) * pt.x + at(1, 1) * pt.y + at(2, 1) * pt.z + at(3, 1) * pt.w;
    const T z = at(0, 2) * pt.x + at(1, 2) * pt.y + at(2, 2) * pt.z + at(3, 2) * pt.w;
    const T w = at(0, 3) * pt.x + at(1, 3) * pt.y + at(2, 3) * pt.z + at(3, 3) * pt.w;
    pResult->set(x, y, z, w);
}

template <class T> inline bool mat4<T>::homogeneous_inverse(const mat4<T> &source) {
    const mat4<T> m = source;
    T det = m.at(0, 0) * (m.at(1, 1) * m.at(2, 2) - m.at(1, 2) * m.at(2, 1)) +
            m.at(1, 0) * (m.at(0, 2) * m.at(2, 1) - m.at(0, 1) * m.at(2, 2)) +
            m.at(2, 0) * (m.at(0, 1) * m.at(1, 2) - m.at(0, 2) * m.at(1, 1));
    if (det == 0)
        return false;
    det = T(1) / det;

    at(0, 0) = (m.at(1, 1) * m.at(2, 2) - m.at(1, 2) * m.at(2, 1)) * det;
    at(0, 1) = (m.at(0, 2) * m.at(2, 1) - m.at(0, 1) * m.at(2, 2)) * det;
    at(0, 2) = (m.at(0, 1) * m.at(1, 2) - m.at(0, 2) * m.at(1, 1)) * det;
    at(0, 3) = -(m.at(0, 3) * at(0, 0) + m.at(1, 3) * at(0, 1) + m.at(2, 3) * at(0, 2));

    at(1, 0) = (m.at(1, 2) * m.at(2, 0) - m.at(1, 0) * m.at(2, 2)) * det;
    at(1, 1) = (m.at(0, 0) * m.at(2, 2) - m.at(0, 2) * m.at(2, 0)) * det;
    at(1, 2) = (m.at(0, 2) * m.at(1, 0) - m.at(0, 0) * m.at(1, 2)) * det;
    at(1, 3) = -(m.at(0, 3) * at(1, 0) + m.at(1, 3) * at(1, 1) + m.at(2, 3) * at(1, 2));

    at(2, 0) = (m.at(1, 0) * m.at(2, 1) - m.at(1, 1) * m.at(2, 0)) * det;
    at(2, 1) = (m.at(0, 1) * m.at(2, 0) - m.at(0, 0) * m.at(2, 1)) * det;
    at(2, 2) = (m.at(0, 0) * m.at(1, 1) - m.at(0, 1) * m.at(1, 0)) * det;
    at(2, 3) = -(m.at(0, 3) * at(2, 0) + m.at(1, 3) * at(2, 1) + m.at(2, 3) * at(2, 2));

    at(3, 0) = at(3, 1) = at(3, 2) = T(0);
    at(3, 3) = T(1);

    return true;
}

template <class T> inline bool invert(mat4<T> *destination, const mat4<T> &m) {
    geometry_detail::output(destination);
    mat4<T> result;
    auto *pRes = &result;
    const T m3344 = m.at(2, 2) * m.at(3, 3) - m.at(3, 2) * m.at(2, 3);
    const T m2344 = m.at(1, 2) * m.at(3, 3) - m.at(3, 2) * m.at(1, 3);
    const T m2334 = m.at(1, 2) * m.at(2, 3) - m.at(2, 2) * m.at(1, 3);
    const T m3244 = m.at(2, 1) * m.at(3, 3) - m.at(3, 1) * m.at(2, 3);
    const T m2244 = m.at(1, 1) * m.at(3, 3) - m.at(3, 1) * m.at(1, 3);
    const T m2234 = m.at(1, 1) * m.at(2, 3) - m.at(2, 1) * m.at(1, 3);
    const T m3243 = m.at(2, 1) * m.at(3, 2) - m.at(3, 1) * m.at(2, 2);
    const T m2243 = m.at(1, 1) * m.at(3, 2) - m.at(3, 1) * m.at(1, 2);
    const T m2233 = m.at(1, 1) * m.at(2, 2) - m.at(2, 1) * m.at(1, 2);
    const T m1344 = m.at(0, 2) * m.at(3, 3) - m.at(3, 2) * m.at(0, 3);
    const T m1334 = m.at(0, 2) * m.at(2, 3) - m.at(2, 2) * m.at(0, 3);
    const T m1244 = m.at(0, 1) * m.at(3, 3) - m.at(3, 1) * m.at(0, 3);
    const T m1234 = m.at(0, 1) * m.at(2, 3) - m.at(2, 1) * m.at(0, 3);
    const T m1243 = m.at(0, 1) * m.at(3, 2) - m.at(3, 1) * m.at(0, 2);
    const T m1233 = m.at(0, 1) * m.at(2, 2) - m.at(2, 1) * m.at(0, 2);
    const T m1324 = m.at(0, 2) * m.at(1, 3) - m.at(1, 2) * m.at(0, 3);
    const T m1224 = m.at(0, 1) * m.at(1, 3) - m.at(1, 1) * m.at(0, 3);
    const T m1223 = m.at(0, 1) * m.at(1, 2) - m.at(1, 1) * m.at(0, 2);

    pRes->at(0, 0) = m.at(1, 1) * m3344 - m.at(2, 1) * m2344 + m.at(3, 1) * m2334;
    pRes->at(1, 0) = -m.at(1, 0) * m3344 + m.at(2, 0) * m2344 - m.at(3, 0) * m2334;
    pRes->at(2, 0) = m.at(1, 0) * m3244 - m.at(2, 0) * m2244 + m.at(3, 0) * m2234;
    pRes->at(3, 0) = -m.at(1, 0) * m3243 + m.at(2, 0) * m2243 - m.at(3, 0) * m2233;

    pRes->at(0, 1) = -m.at(0, 1) * m3344 + m.at(2, 1) * m1344 - m.at(3, 1) * m1334;
    pRes->at(1, 1) = m.at(0, 0) * m3344 - m.at(2, 0) * m1344 + m.at(3, 0) * m1334;
    pRes->at(2, 1) = -m.at(0, 0) * m3244 + m.at(2, 0) * m1244 - m.at(3, 0) * m1234;
    pRes->at(3, 1) = m.at(0, 0) * m3243 - m.at(2, 0) * m1243 + m.at(3, 0) * m1233;

    pRes->at(0, 2) = m.at(0, 1) * m2344 - m.at(1, 1) * m1344 + m.at(3, 1) * m1324;
    pRes->at(1, 2) = -m.at(0, 0) * m2344 + m.at(1, 0) * m1344 - m.at(3, 0) * m1324;
    pRes->at(2, 2) = m.at(0, 0) * m2244 - m.at(1, 0) * m1244 + m.at(3, 0) * m1224;
    pRes->at(3, 2) = -m.at(0, 0) * m2243 + m.at(1, 0) * m1243 - m.at(3, 0) * m1223;

    pRes->at(0, 3) = -m.at(0, 1) * m2334 + m.at(1, 1) * m1334 - m.at(2, 1) * m1324;
    pRes->at(1, 3) = m.at(0, 0) * m2334 - m.at(1, 0) * m1334 + m.at(2, 0) * m1324;
    pRes->at(2, 3) = -m.at(0, 0) * m2234 + m.at(1, 0) * m1234 - m.at(2, 0) * m1224;
    pRes->at(3, 3) = m.at(0, 0) * m2233 - m.at(1, 0) * m1233 + m.at(2, 0) * m1223;

    T fDet = m.at(0, 0) * pRes->at(0, 0) + m.at(1, 0) * pRes->at(0, 1) + m.at(2, 0) * pRes->at(0, 2) +
             m.at(3, 0) * pRes->at(0, 3);
    if (fDet == 0)
        return false;
    fDet = T(1) / fDet;
    pRes->at(0, 0) *= fDet;
    pRes->at(1, 0) *= fDet;
    pRes->at(2, 0) *= fDet;
    pRes->at(3, 0) *= fDet;
    pRes->at(0, 1) *= fDet;
    pRes->at(1, 1) *= fDet;
    pRes->at(2, 1) *= fDet;
    pRes->at(3, 1) *= fDet;
    pRes->at(0, 2) *= fDet;
    pRes->at(1, 2) *= fDet;
    pRes->at(2, 2) *= fDet;
    pRes->at(3, 2) *= fDet;
    pRes->at(0, 3) *= fDet;
    pRes->at(1, 3) *= fDet;
    pRes->at(2, 3) *= fDet;
    pRes->at(3, 3) *= fDet;

    *destination = result;
    return true;
}

template <class T> inline T det(T d00) {
    return d00;
}
template <class T> inline T det(T d00, T d01, T d10, T d11) {
    return d00 * d11 - d01 * d10;
}

template <class T> inline T det(T d00, T d01, T d02, T d10, T d11, T d12, T d20, T d21, T d22) {
    return d00 * (d11 * d22 - d21 * d12) - d10 * (d01 * d22 - d21 * d02) + d20 * (d01 * d12 - d11 * d02);
}

template <class T>
inline T det(T d00, T d01, T d02, T d03, T d10, T d11, T d12, T d13, T d20, T d21, T d22, T d23, T d30, T d31,
             T d32, T d33) {
    const T M3344 = d22 * d33 - d32 * d23;
    const T M2344 = d12 * d33 - d32 * d13;
    const T M2334 = d12 * d23 - d22 * d13;
    const T M1344 = d02 * d33 - d32 * d03;
    const T M1334 = d02 * d23 - d22 * d03;
    const T M1324 = d02 * d13 - d12 * d03;

    return d00 * (d11 * M3344 - d21 * M2344 + d31 * M2334) - d10 * (d01 * M3344 - d21 * M1344 + d31 * M1334) +
           d20 * (d01 * M2344 - d11 * M1344 + d31 * M1324) - d30 * (d01 * M2334 - d11 * M1334 + d21 * M1324);
}
template <class T> inline T det(const mat4<T> &m) {
    return det(m.at(0, 0), m.at(0, 1), m.at(0, 2), m.at(0, 3), m.at(1, 0), m.at(1, 1), m.at(1, 2), m.at(1, 3),
               m.at(2, 0), m.at(2, 1), m.at(2, 2), m.at(2, 3), m.at(3, 0), m.at(3, 1), m.at(3, 2),
               m.at(3, 3));
}

template <class T> inline void quat<T>::from_angle_axis(T fAngle, const vec3<T> &vAxis, bool bNormalizeAxis) {
    if (!(bNormalizeAxis || length(length(vAxis) - 1) < geometry_detail::tolerance<T>("0.0001")))
        throw std::domain_error("rotation axis must be normalized");

    fAngle *= T::from_string("0.5");
    const T fSinAlpha = bNormalizeAxis ? sin(fAngle) / length(vAxis) : sin(fAngle);
    components_.x = vAxis.x * fSinAlpha;
    components_.y = vAxis.y * fSinAlpha;
    components_.z = vAxis.z * fSinAlpha;
    components_.w = cos(fAngle);
}
template <class T>
inline void quat<T>::from_angle_axis(T fAngle, T fAxisX, T fAxisY, T fAxisZ, bool bNormalizeAxis) {
    if (!(bNormalizeAxis ||
          length(length(fAxisX, fAxisY, fAxisZ) - 1) < geometry_detail::tolerance<T>("0.0001")))
        throw std::domain_error("rotation axis must be normalized");

    fAngle *= T::from_string("0.5");
    const T fSinAlpha = bNormalizeAxis ? sin(fAngle) / length(fAxisX, fAxisY, fAxisZ) : sin(fAngle);
    components_.x = fAxisX * fSinAlpha;
    components_.y = fAxisY * fSinAlpha;
    components_.z = fAxisZ * fSinAlpha;
    components_.w = cos(fAngle);
}

template <class T> inline void quat<T>::from_euler_matrix(const mat4<T> &m) {

    T qs2 = T::from_string("0.25") * (m.at(0, 0) + m.at(1, 1) + m.at(2, 2) + 1);
    T qx2 = qs2 - T::from_string("0.5") * (m.at(1, 1) + m.at(2, 2));
    T qy2 = qs2 - T::from_string("0.5") * (m.at(2, 2) + m.at(0, 0));
    T qz2 = qs2 - T::from_string("0.5") * (m.at(0, 0) + m.at(1, 1));

    int n = (qs2 > qx2) ? ((qs2 > qy2) ? ((qs2 > qz2) ? 0 : 3) : ((qy2 > qz2) ? 2 : 3))
                        : ((qx2 > qy2) ? ((qx2 > qz2) ? 1 : 3) : ((qy2 > qz2) ? 2 : 3));

    T tmp;
    switch (n) {
    case 0:
        components_.w = static_cast<T>(sqrt(qs2));
        tmp = T::from_string("0.25") / components_.w;
        components_.x = (m.at(2, 1) - m.at(1, 2)) * tmp;
        components_.y = (m.at(0, 2) - m.at(2, 0)) * tmp;
        components_.z = (m.at(1, 0) - m.at(0, 1)) * tmp;
        break;
    case 1:
        components_.x = static_cast<T>(sqrt(qx2));
        tmp = T::from_string("0.25") / components_.x;
        components_.w = (m.at(2, 1) - m.at(1, 2)) * tmp;
        components_.y = (m.at(0, 1) + m.at(1, 0)) * tmp;
        components_.z = (m.at(0, 2) + m.at(2, 0)) * tmp;
        break;
    case 2:
        components_.y = static_cast<T>(sqrt(qy2));
        tmp = T::from_string("0.25") / components_.y;
        components_.w = (m.at(0, 2) - m.at(2, 0)) * tmp;
        components_.z = (m.at(1, 2) + m.at(2, 1)) * tmp;
        components_.x = (m.at(1, 0) + m.at(0, 1)) * tmp;
        break;
    case 3:
        components_.z = static_cast<T>(sqrt(qz2));
        tmp = T::from_string("0.25") / components_.z;
        components_.w = (m.at(1, 0) - m.at(0, 1)) * tmp;
        components_.x = (m.at(2, 0) + m.at(0, 2)) * tmp;
        components_.y = (m.at(2, 1) + m.at(1, 2)) * tmp;
        break;
    }

    minimize_rotation_angle();

    fxp::normalize(components_.x, components_.y, components_.z, components_.w);
}

template <class T> inline void quat<T>::from_euler_angles(T yaw, T pitch, T roll) {
    const T fHalfYaw = yaw * T::from_string("0.5");
    const T fHalfPitch = pitch * T::from_string("0.5");
    const T fHalfRoll = roll * T::from_string("0.5");

    const T fCosYaw = cos(fHalfYaw);
    const T fSinYaw = sin(fHalfYaw);
    const T fCosPitch = cos(fHalfPitch);
    const T fSinPitch = sin(fHalfPitch);
    const T fCosRoll = cos(fHalfRoll);
    const T fSinRoll = sin(fHalfRoll);

    components_.x = fSinRoll * fCosPitch * fCosYaw - fCosRoll * fSinPitch * fSinYaw;
    components_.y = fCosRoll * fSinPitch * fCosYaw + fSinRoll * fCosPitch * fSinYaw;
    components_.z = fCosRoll * fCosPitch * fSinYaw - fSinRoll * fSinPitch * fCosYaw;
    components_.w = fCosRoll * fCosPitch * fCosYaw + fSinRoll * fSinPitch * fSinYaw;
}

template <class T> inline quat<T>::quat(T fAngle, T fAxisX, T fAxisY, T fAxisZ, bool bNormalizeAxis) {
    from_angle_axis(fAngle, fAxisX, fAxisY, fAxisZ, bNormalizeAxis);
}

template <class T> inline quat<T>::quat(T fAngle, const vec3<T> &vAxis, bool bNormalizeAxis) {
    from_angle_axis(fAngle, vAxis, bNormalizeAxis);
}

template <class T> inline void quat<T>::deriv(const quat<T> &q, const vec3<T> &v) {
    components_.x =
        T::from_string("0.5") * (q.components_.w * v.x - q.components_.z * v.y + q.components_.y * v.z);
    components_.y =
        T::from_string("0.5") * (q.components_.z * v.x + q.components_.w * v.y - q.components_.x * v.z);
    components_.z =
        T::from_string("0.5") * (-q.components_.y * v.x + q.components_.x * v.y + q.components_.w * v.z);
    components_.w =
        T::from_string("0.5") * (-q.components_.x * v.x - q.components_.y * v.y - q.components_.z * v.z);
}

template <class T> inline const vec3<T> quat<T>::get_x_axis() const {
    return vec3<T>(
        components_.w * components_.w -
            (components_.x * components_.x + components_.y * components_.y + components_.z * components_.z) +
            T(2) * components_.x * components_.x,
        (components_.z * components_.w + components_.x * components_.y) * T(2),
        (-components_.y * components_.w + components_.x * components_.z) * T(2));
}
template <class T> inline const vec3<T> quat<T>::get_y_axis() const {
    return vec3<T>(
        (-components_.z * components_.w + components_.y * components_.x) * T(2),
        components_.w * components_.w -
            (components_.x * components_.x + components_.y * components_.y + components_.z * components_.z) +
            T(2) * components_.y * components_.y,
        (components_.x * components_.w + components_.y * components_.z) * T(2));
}
template <class T> inline const vec3<T> quat<T>::get_z_axis() const {
    return vec3<T>(
        (components_.y * components_.w + components_.z * components_.x) * T(2),
        (-components_.x * components_.w + components_.z * components_.y) * T(2),
        components_.w * components_.w -
            (components_.x * components_.x + components_.y * components_.y + components_.z * components_.z) +
            T(2) * components_.z * components_.z);
}
template <class T> inline void quat<T>::get_x_axis(vec3<T> *pRes) const {
    pRes->x =
        components_.w * components_.w -
        (components_.x * components_.x + components_.y * components_.y + components_.z * components_.z) +
        T(2) * components_.x * components_.x;
    pRes->y = (components_.z * components_.w + components_.x * components_.y) * T(2);
    pRes->z = (-components_.y * components_.w + components_.x * components_.z) * T(2);
}
template <class T> inline void quat<T>::get_y_axis(vec3<T> *pRes) const {
    pRes->x = (-components_.z * components_.w + components_.y * components_.x) * T(2);
    pRes->y =
        components_.w * components_.w -
        (components_.x * components_.x + components_.y * components_.y + components_.z * components_.z) +
        T(2) * components_.y * components_.y;
    pRes->z = (components_.x * components_.w + components_.y * components_.z) * T(2);
}
template <class T> inline void quat<T>::get_z_axis(vec3<T> *pRes) const {
    pRes->x = (components_.y * components_.w + components_.z * components_.x) * T(2);
    pRes->y = (-components_.x * components_.w + components_.z * components_.y) * T(2);
    pRes->z =
        components_.w * components_.w -
        (components_.x * components_.x + components_.y * components_.y + components_.z * components_.z) +
        T(2) * components_.z * components_.z;
}
template <class T> inline const vec3<T> quat<T>::rotate(const vec3<T> &r) const {
    const vec3<T> L(components_.x, components_.y, components_.z);
    return (r * (components_.w * components_.w - L * L) + (T(2) * components_.w) * (L ^ r) +
            (T(2) * (L * r)) * L);
}
template <class T> inline void quat<T>::rotate(vec3<T> *pRes, const vec3<T> &vec) const {
    const vec3<T> L(components_.x, components_.y, components_.z);
    *pRes = (vec * (components_.w * components_.w - L * L) + (T(2) * components_.w) * (L ^ vec) +
             (T(2) * (L * vec)) * L);
}

template <class T = real> inline const quat<T> operator*(const quat<T> &a, const quat<T> &b) {
    return quat<T>(a.components_.w * b.components_.x + b.components_.w * a.components_.x +
                       (a.components_.y * b.components_.z - a.components_.z * b.components_.y),
                   a.components_.w * b.components_.y + b.components_.w * a.components_.y +
                       (a.components_.z * b.components_.x - a.components_.x * b.components_.z),
                   a.components_.w * b.components_.z + b.components_.w * a.components_.z +
                       (a.components_.x * b.components_.y - a.components_.y * b.components_.x),
                   a.components_.w * b.components_.w -
                       (a.components_.x * b.components_.x + a.components_.y * b.components_.y +
                        a.components_.z * b.components_.z),
                   0, 0);
}

template <class T> inline const quat<T> operator/(const quat<T> &a, const quat<T> &b) {
    quat<T> q1;
    q1.unit_inverse(b);
    return q1 * a;
}

template <class T> inline quat<T> &quat<T>::operator*=(const quat<T> &a) {
    T xtmp = a.components_.w * components_.x + components_.w * a.components_.x +
             (a.components_.y * components_.z - a.components_.z * components_.y);
    T ytmp = a.components_.w * components_.y + components_.w * a.components_.y +
             (a.components_.z * components_.x - a.components_.x * components_.z);
    T ztmp = a.components_.w * components_.z + components_.w * a.components_.z +
             (a.components_.x * components_.y - a.components_.y * components_.x);
    T wtmp =
        a.components_.w * components_.w -
        (a.components_.x * components_.x + a.components_.y * components_.y + a.components_.z * components_.z);
    components_.x = xtmp;
    components_.y = ytmp;
    components_.z = ztmp;
    components_.w = wtmp;

    return *this;
}

template <class T> inline quat<T> &quat<T>::operator/=(const quat<T> &q) {
    quat<T> q1;
    q1.unit_inverse(q);
    (*this) *= q1;
    return *this;
}

template <class T> inline bool quat<T>::inverse(const quat<T> &q) {
    T norm = length_squared(q.components_.x, q.components_.y, q.components_.z, q.components_.w);
    if (norm > T::epsilon()) {
        norm = T(1) / norm;
        components_.x = -q.components_.x * norm;
        components_.y = -q.components_.y * norm;
        components_.z = -q.components_.z * norm;
        components_.w = q.components_.w * norm;
        return true;
    } else
        return false;
}

template <class T> inline bool quat<T>::inverse() {
    T norm = length_squared(components_.x, components_.y, components_.z, components_.w);
    if (norm > T::epsilon()) {
        norm = T(1) / norm;
        components_.x *= -norm;
        components_.y *= -norm;
        components_.z *= -norm;
        components_.w *= norm;
        return true;
    } else
        return false;
}

template <class T> inline void quat<T>::decomp_angle_axis(T *pfAngle, vec3<T> *pvAxis) const {

    T len = length_squared(components_.x, components_.y, components_.z);
    if (len > geometry_detail::tolerance<T>("1e-8")) {
        *pfAngle = T(2) * acos(geometry_detail::clamp(components_.w, T(-1), T(1)));
        len = T(1) / T(sqrt(len));
        pvAxis->x = components_.x * len;
        pvAxis->y = components_.y * len;
        pvAxis->z = components_.z * len;
    } else {

        *pfAngle = T(0);
        pvAxis->x = T(1);
        pvAxis->y = T(0);
        pvAxis->z = T(0);
    }
}
template <class T>
inline void quat<T>::decomp_angle_axis(T *pfAngle, T *pfAxisX, T *pfAxisY, T *pfAxisZ) const {

    T len = components_.x * components_.x + components_.y * components_.y + components_.z * components_.z;
    if (len > geometry_detail::tolerance<T>("1e-8")) {
        *pfAngle = T(2) * acos(geometry_detail::clamp(components_.w, T(-1), T(1)));
        len = T(1) / T(sqrt(len));
        *pfAxisX = components_.x * len;
        *pfAxisY = components_.y * len;
        *pfAxisZ = components_.z * len;
    } else {

        *pfAngle = T(0);
        *pfAxisX = T(1);
        *pfAxisY = T(0);
        *pfAxisZ = T(0);
    }
}

template <class T> inline void quat<T>::decomp_euler_matrix(mat4<T> *pRes) const {
    const T tx = components_.x + components_.x;
    const T ty = components_.y + components_.y;
    const T tz = components_.z + components_.z;
    const T twx = tx * components_.w;
    const T twy = ty * components_.w;
    const T twz = tz * components_.w;
    const T txx = tx * components_.x;
    const T txy = ty * components_.x;
    const T txz = tz * components_.x;
    const T tyy = ty * components_.y;
    const T tyz = tz * components_.y;
    const T tzz = tz * components_.z;

    pRes->at(0, 0) = T(1) - (tyy + tzz);
    pRes->at(0, 1) = txy - twz;
    pRes->at(0, 2) = txz + twy;

    pRes->at(1, 0) = txy + twz;
    pRes->at(1, 1) = T(1) - (txx + tzz);
    pRes->at(1, 2) = tyz - twx;

    pRes->at(2, 0) = txz - twy;
    pRes->at(2, 1) = tyz + twx;
    pRes->at(2, 2) = T(1) - (txx + tyy);
}

template <class T> inline void quat<T>::decomp_reversed_euler_matrix(mat4<T> *pRes) const {
    const T tx = -(components_.x + components_.x);
    const T ty = -(components_.y + components_.y);
    const T tz = -(components_.z + components_.z);
    const T twx = tx * components_.w;
    const T twy = ty * components_.w;
    const T twz = tz * components_.w;
    const T txx = -tx * components_.x;
    const T txy = -ty * components_.x;
    const T txz = -tz * components_.x;
    const T tyy = -ty * components_.y;
    const T tyz = -tz * components_.y;
    const T tzz = -tz * components_.z;

    pRes->at(0, 0) = T(1) - (tyy + tzz);
    pRes->at(0, 1) = txy - twz;
    pRes->at(0, 2) = txz + twy;

    pRes->at(1, 0) = txy + twz;
    pRes->at(1, 1) = T(1) - (txx + tzz);
    pRes->at(1, 2) = tyz - twx;

    pRes->at(2, 0) = txz - twy;
    pRes->at(2, 1) = tyz + twx;
    pRes->at(2, 2) = T(1) - (txx + tyy);
}

template <class T> inline const quat<T> quat<T>::exp() const {

    const T angle = length(components_.x, components_.y, components_.z);
    const T sn = sin(angle);
    quat<T> result;

    result.components_.w = T(cos(angle));

    if (length(sn) >= geometry_detail::tolerance<T>("0.0001")) {
        const T coeff = T(sn / angle);
        result.components_.x = coeff * components_.x;
        result.components_.y = coeff * components_.y;
        result.components_.z = coeff * components_.z;
    } else {
        result.components_.x = components_.x;
        result.components_.y = components_.y;
        result.components_.z = components_.z;
    }

    return result;
}

template <class T> inline const quat<T> quat<T>::log() const {

    if (length(components_.w) < T(1)) {
        const T angle = acos(geometry_detail::clamp(components_.w, T(-1), T(1)));
        const T sn = sin(angle);
        if (length(sn) >= geometry_detail::tolerance<T>("0.0001")) {
            const T coeff = T(angle / sn);
            return quat<T>(coeff * components_.x, coeff * components_.y, coeff * components_.z, 0, 0, 0);
        }
    }

    return quat<T>(components_.x, components_.y, components_.z, 0, 0, 0);
}

template <class T> inline void quat<T>::slerp(const T factor, const quat<T> &p, const quat<T> &q) {
    T scale0, scale1;
    quat<T> q1(q);

    T cosom = p.components_.x * q.components_.x + p.components_.y * q.components_.y +
              p.components_.z * q.components_.z + p.components_.w * q.components_.w;

    cosom = geometry_detail::clamp(cosom, T(-1), T(1));
    if (cosom < 0) {
        cosom = -cosom;
        q1.negate(q);
    }

    if ((T(1) - cosom) > geometry_detail::tolerance<T>("0.0001")) {
        const T omega = acos(cosom);
        const T sinom = static_cast<T>(T(1) / sin(omega));
        scale0 = T(sin((T(1) - factor) * omega) * sinom);
        scale1 = T(sin(factor * omega) * sinom);
    } else {
        scale0 = T(1) - factor;
        scale1 = factor;
    }

    components_.x = scale0 * p.components_.x + scale1 * q1.components_.x;
    components_.y = scale0 * p.components_.y + scale1 * q1.components_.y;
    components_.z = scale0 * p.components_.z + scale1 * q1.components_.z;
    components_.w = scale0 * p.components_.w + scale1 * q1.components_.w;
}

template <class T>
inline void get_circles_by_tangent(const vec2<T> &tang, const vec2<T> &p, const T r, circle<T> *c1,
                                   circle<T> *c2) {
    const vec2<T> v(-tang.y, tang.x);

    c1->radius = r;
    c1->center = p + v * r;

    c2->radius = r;
    c2->center = p - v * r;
}

template <class T>
inline bool find_tangent_points(const vec2<T> &p, const circle<T> &c, vec2<T> *p1, vec2<T> *p2) {
    const vec2<T> v = c.center - p;
    const T hyp2 = length_squared(v);
    const T r2 = length_squared(c.radius);

    if (hyp2 < r2)
        return false;

    if (hyp2 == r2) {
        *p1 = p;
        *p2 = p;
    } else {
        const T leg2 = hyp2 - r2;
        const T cossin = T(sqrt(leg2)) * c.radius / hyp2;
        const T cos2 = leg2 / hyp2;

        p1->x = v.x * cos2 - v.y * cossin + p.x;
        p1->y = v.x * cossin + v.y * cos2 + p.y;

        p2->x = p1->x + 2 * v.y * cossin;
        p2->y = p1->y - 2 * v.x * cossin;
    }

    return true;
}

template <class T> inline void line2<T>::project_point(const vec2<T> &point, vec2<T> *result) {
    const auto n = length_squared(a, b);
    if (!n)
        throw std::domain_error("line normal is zero or too small to project");
    const auto k = (a * point.x + b * point.y + c) / n;
    geometry_detail::output(result) = vec2<T>(point.x - k * a, point.y - k * b);
}

template <class T> inline T line2<T>::dist_to_point(const vec2<T> &point) {
    const auto n = length(a, b);
    if (!n)
        throw std::domain_error("zero line normal");
    return (a * point.x + b * point.y + c) / n;
}

template <class T> inline T triangle_area2(const vec2<T> &p1, const vec2<T> &p2, const vec2<T> &p3) {
    return p1.x * (p2.y - p3.y) + p2.x * (p3.y - p1.y) + p3.x * (p1.y - p2.y);
}

template <class T>
inline bool is_point_inside_triangle(const vec2<T> &p1, const vec2<T> &p2, const vec2<T> &p3,
                                     const vec2<T> &p) {
    int nSign1 = geometry_detail::sign(triangle_area2(p, p1, p2));
    int nSign2 = geometry_detail::sign(triangle_area2(p, p2, p3));
    int nSign3 = geometry_detail::sign(triangle_area2(p, p3, p1));
    int nSign = geometry_detail::sign(triangle_area2(p1, p2, p3));

    if (nSign != 0)
        return nSign * nSign1 >= 0 && nSign * nSign2 >= 0 && nSign * nSign3 >= 0;

    else {
        if (nSign1 == 0 && nSign2 == 0 && nSign3 == 0) {
            return segment2<T>(p1, p2).get_dist_to_point(p) < geometry_detail::tolerance<T>("1e-6") ||
                   segment2<T>(p1, p3).get_dist_to_point(p) < geometry_detail::tolerance<T>("1e-6");
        } else
            return false;
    }
}

template <class T> inline T segment2<T>::get_dist_to_point(const vec2<T> &point) const {
    if ((point - p1) * direction <= 0)
        return length(point - p1);
    else if ((point - p2) * direction >= 0)
        return length(point - p2);
    else
        return length(triangle_area2(p1, p2, point)) / length(p2 - p1);
}

template <class T> inline void segment2<T>::get_closest_point(const vec2<T> &point, vec2<T> *result) const {
    if ((point - p1) * direction <= 0)
        *result = p1;
    else if ((point - p2) * direction >= 0)
        *result = p2;
    else
        line2<T>(p1, p2).project_point(point, result);
}

template <class T> inline void make_matrix(mat4<T> *pMatrix, const vec3<T> &pos, const quat<T> &rot) {
    pMatrix->set(pos, rot);
}

template <class T>
inline void make_matrix(mat4<T> *pMatrix, const vec3<T> &pos, const quat<T> &rot, const vec3<T> &scale) {
    rot.decomp_euler_matrix(pMatrix);
    pMatrix->at(0, 0) *= scale.x;
    pMatrix->at(1, 0) *= scale.x;
    pMatrix->at(2, 0) *= scale.x;
    pMatrix->at(0, 1) *= scale.y;
    pMatrix->at(1, 1) *= scale.y;
    pMatrix->at(2, 1) *= scale.y;
    pMatrix->at(0, 2) *= scale.z;
    pMatrix->at(1, 2) *= scale.z;
    pMatrix->at(2, 2) *= scale.z;
    pMatrix->at(0, 3) = pos.x;
    pMatrix->at(1, 3) = pos.y;
    pMatrix->at(2, 3) = pos.z;
    pMatrix->at(3, 0) = pMatrix->at(3, 1) = pMatrix->at(3, 2) = T(0);
    pMatrix->at(3, 3) = T(1);
}

template <class T> inline std::uint16_t direction_by_vector(const vec2<T> &vec);
template <class T> inline std::uint16_t direction_by_vector(T x, T y);
template <class T = real> inline const vec2<T> vector_by_direction(std::uint16_t direction);
template <class T> inline std::uint16_t z_direction(const T x, const T y, const T z);
template <class T> inline std::uint16_t z_direction(const vec2<T> &vec, const T z);

template <class T> inline std::uint16_t z_angle(const T x, const T y, T z);

template <class T> inline std::uint16_t z_angle(const vec2<T> &vec, const T z);
template <class T> inline std::uint16_t z_angle(const vec3<T> &vPoint);
inline std::uint16_t direction_difference(std::uint16_t dir1, std::uint16_t dir2);
inline int direction_difference_sign(std::uint16_t dir1, std::uint16_t dir2);

inline bool is_in_the_angle(std::uint16_t direction, std::uint16_t startAngleDir,
                            std::uint16_t finishAngleDir);

inline bool is_in_the_min_angle(std::uint16_t direction, std::uint16_t dir1, std::uint16_t dir2);

template <class T> inline bool is_almost_zero(const vec2<T> &vec) {
    const static T eps = T::from_string("0.0001");
    return (length(vec.x) < eps && length(vec.y) < eps);
}

template <class T> inline bool is_almost_zero(const T x, const T y) {
    const static T eps = T::from_string("0.0001");
    return (length(x) < eps && length(y) < eps);
}

enum side {
    front = 0,
    left = 1,
    back = 2,
    right = 3,
    top = 4,
    bottom = 5,
};

template <class T = real> struct oriented_rect {
    std::array<vec2<T>, 4> corners{};

    vec2<T> &corner0() noexcept { return corners[0]; }
    vec2<T> &corner1() noexcept { return corners[1]; }
    vec2<T> &corner2() noexcept { return corners[2]; }
    vec2<T> &corner3() noexcept { return corners[3]; }
    const vec2<T> &corner0() const noexcept { return corners[0]; }
    const vec2<T> &corner1() const noexcept { return corners[1]; }
    const vec2<T> &corner2() const noexcept { return corners[2]; }
    const vec2<T> &corner3() const noexcept { return corners[3]; }
    vec2<T> direction, perpendicular, center;
    T length_ahead, length_back, width;

    bool is_intersect_project(const vec2<T> &v1, const vec2<T> &v2, const vec2<T> &v3, const vec2<T> &v4,
                              const vec2<T> &direction, const T min, const T max) const;

    void init_rect(const vec2<T> &_v1, const vec2<T> &_v2, const vec2<T> &_v3, const vec2<T> &_v4);

    void init_rect(const vec2<T> &center, const vec2<T> &direction, const T length, const T width);

    void init_rect(const vec2<T> &center, const vec2<T> &direction, const T length_ahead, const T length_back,
                   const T width);

    bool is_intersected(const oriented_rect<T> &rect) const;
    bool is_intersected(const segment2<T> &segment) const;

    bool is_point_inside(const vec2<T> &point) const;
    bool is_intersect_circle(const vec2<T> &circleCenter, const T r) const;
    bool is_intersect_circle(const circle<T> &circle) const {
        return is_intersect_circle(circle.center, circle.radius);
    }
    bool is_intersect_triangle(const vec2<T> &v1, const vec2<T> &v2, const vec2<T> &v3) const;

    int get_side(std::uint16_t dirFromRectCenter) const;
    int get_side(const vec2<T> &point) const;

    void compress(const T fFactor);

    oriented_rect() : length_ahead(0), length_back(0), width(0) {}
};

template <class T> inline T projected_distance(const oriented_rect<T> rect1, const oriented_rect<T> rect2);

template <class T> inline std::uint16_t visible_angle(const vec2<T> &point, const oriented_rect<T> rect);

template <class T>
inline bool intersect_ray_rect(vec2<T> *pvResult, const vec2<T> &vPoint, const vec2<T> &vDir,
                               const oriented_rect<T> &rect);

template <class T> inline void get_angles(const vec3<T> &vNormal, T *pfPhi, T *pfTheta) {

    {
        const T fLen2 = length_squared(vNormal.y, vNormal.z);
        *pfPhi = fLen2 < geometry_detail::tolerance<T>("1e-8") ? 0 : vNormal.z / sqrt(fLen2);
        *pfPhi = -geometry_detail::sign(vNormal.y) * acos(geometry_detail::clamp(*pfPhi, -T(1), T(1)));
    }

    {
        const T fLen2 = length_squared(vNormal.x, vNormal.z);
        *pfTheta = fLen2 < geometry_detail::tolerance<T>("1e-8") ? 0 : vNormal.z / sqrt(fLen2);
        *pfTheta = geometry_detail::sign(vNormal.x) * acos(geometry_detail::clamp(*pfTheta, -T(1), T(1)));
    }
}

template <class T> inline void make_orientation(quat<T> *pQuat, const vec3<T> &vNormal) {
    T fPhi, fTheta;
    get_angles(vNormal, &fPhi, &fTheta);

    quat<T> q1(fPhi, axis3_x_v<T>);
    quat<T> q2(fTheta, axis3_y_v<T>);
    q1.minimize_rotation_angle();
    q2.minimize_rotation_angle();
    (*pQuat) *= q1 * q2;
}

template <class T> inline void quat<T>::decomp_euler_angles(T *pfYaw, T *pfPitch, T *pfRoll) {
    const T x2 = components_.x * components_.x;
    const T y2 = components_.y * components_.y;
    const T z2 = components_.z * components_.z;
    const T w2 = components_.w * components_.w;

    *pfYaw = atan2(2 * (components_.w * components_.z + components_.x * components_.y), w2 - z2 + x2 - y2);
    *pfPitch = -asin(geometry_detail::clamp(
        T(2) * (components_.z * components_.x - components_.w * components_.y), T(-1), T(1)));
    *pfRoll = atan2(2 * (components_.z * components_.y + components_.w * components_.x), w2 + z2 - x2 - y2);
}

template <class T> inline std::uint16_t direction_by_vector(T x, T y) {
    if (is_almost_zero(x, y))
        return 0;
    auto ax = detail::magnitude(x.raw_value()), ay = detail::magnitude(y.raw_value());
    std::uint32_t base = 49152;
    if (x <= 0 && y > 0) {
        base = 0;
        std::swap(ax, ay);
    } else if (y <= 0 && x < 0)
        base = 16384;
    else if (x >= 0 && y < 0) {
        base = 32768;
        std::swap(ax, ay);
    }
    const auto denominator = detail::add({0, ax}, {0, ay});
    const auto fraction = detail::div_portable(detail::shl({0, ay}, 14), denominator).quotient.lo;
    return static_cast<std::uint16_t>(base + fraction);
}

template <class T> inline std::uint16_t direction_by_vector(const vec2<T> &vec) {
    return direction_by_vector(vec.x, vec.y);
}

template <class T> inline const vec2<T> vector_by_direction(std::uint16_t direction) {
    const T fDir = T(direction % 16384) / T(16384);
    vec2<T> result(1 - fDir, fDir);

    if (direction < 16384) {
        result.y = -result.y;
        std::swap(result.x, result.y);
    } else if (direction < 32768) {
        result.x = -result.x;
        result.y = -result.y;
    } else if (direction < 49152) {
        result.x = -result.x;
        std::swap(result.x, result.y);
    }

    normalize(&result);
    return result;
}

inline std::uint16_t direction_difference(std::uint16_t dir1, std::uint16_t dir2) {
    const std::uint16_t clockWise = static_cast<std::uint16_t>(dir1 - dir2);
    const std::uint16_t antiClockWise = static_cast<std::uint16_t>(dir2 - dir1);

    return (std::min)(clockWise, antiClockWise);
}

inline int direction_difference_sign(std::uint16_t dir1, std::uint16_t dir2) {
    const std::uint16_t clockWise = static_cast<std::uint16_t>(dir1 - dir2);
    const std::uint16_t antiClockWise = static_cast<std::uint16_t>(dir2 - dir1);

    return geometry_detail::sign(int(antiClockWise) - int(clockWise));
}

inline bool is_in_the_angle(std::uint16_t direction, std::uint16_t startAngleDir,
                            std::uint16_t finishAngleDir) {
    return std::uint16_t(direction - startAngleDir) + std::uint16_t(finishAngleDir - direction) ==
           std::uint16_t(finishAngleDir - startAngleDir);
}

inline bool is_in_the_min_angle(std::uint16_t direction, std::uint16_t dir1, std::uint16_t dir2) {
    return (int)direction_difference(direction, dir1) + (int)direction_difference(direction, dir2) ==
           direction_difference(dir1, dir2);
}

template <class T> inline std::uint16_t z_direction(const T x, const T y, const T z) {
    return direction_by_vector(length(x, y), z);
}

template <class T> inline std::uint16_t z_direction(const vec2<T> &vec, const T z) {
    return z_direction(vec.x, vec.y, z);
}

template <class T> inline std::uint16_t z_angle(const vec2<T> &vec, const T z) {
    return z_angle(vec.x, vec.y, z);
}

template <class T> inline std::uint16_t z_angle(const T x, const T y, const T z) {
    std::uint16_t wZDir = z_direction(x, y, z);

    std::uint16_t wZAngle =
        (std::min)(direction_difference(wZDir, 16384 * 3), direction_difference(wZDir, 16384));
    return z >= 0 ? wZAngle : static_cast<std::uint16_t>(UINT32_C(65536) - wZAngle);
}

template <class T> inline std::uint16_t z_angle(const vec3<T> &vPoint) {
    return z_angle(vPoint.x, vPoint.y, vPoint.z);
}

template <class T>
inline void oriented_rect<T>::init_rect(const vec2<T> &_v1, const vec2<T> &_v2, const vec2<T> &_v3,
                                        const vec2<T> &_v4) {
    corners[0] = _v1;
    corners[1] = _v2;
    corners[2] = _v3;
    corners[3] = _v4;
    center = (corner0() + corner2()) * T::from_string("0.5");

    direction = corner3() - corner0();

    length_back = length_ahead = length(direction) / 2;
    normalize(&direction);

    perpendicular.x = -direction.y;
    perpendicular.y = direction.x;

    width = length(corner1() - corner0()) * T::from_string("0.5");
}

template <class T>
inline void oriented_rect<T>::init_rect(const vec2<T> &_center, const vec2<T> &_dir, const T length,
                                        const T _width) {
    if (!length_squared(_dir)) {
        if (_dir == zero2_v<T>)
            throw std::domain_error("rectangle direction is zero");
    }
    center = _center;
    direction = _dir;
    normalize(&direction);

    perpendicular.x = -direction.y;
    perpendicular.y = direction.x;

    length_back = length_ahead = length;
    width = _width;

    const vec2<T> pointBack = center - direction * length;
    const vec2<T> pointForward = center + direction * length;

    corners[0] = pointBack - perpendicular * width;
    corners[1] = pointBack + perpendicular * width;
    corners[2] = pointForward + perpendicular * width;
    corners[3] = pointForward - perpendicular * width;
}

template <class T>
inline void oriented_rect<T>::init_rect(const vec2<T> &_center, const vec2<T> &_dir, const T _lengthAhead,
                                        const T _lengthBack, const T _width) {
    if (!length_squared(_dir)) {
        if (_dir == zero2_v<T>)
            throw std::domain_error("rectangle direction is zero");
    }
    center = _center;
    direction = _dir;
    normalize(&direction);
    perpendicular.x = -direction.y;
    perpendicular.y = direction.x;

    length_back = _lengthBack;
    length_ahead = _lengthAhead;
    width = _width;

    const vec2<T> pointBack = center - direction * length_back;
    const vec2<T> pointForward = center + direction * length_ahead;

    corners[0] = pointBack - perpendicular * width;
    corners[1] = pointBack + perpendicular * width;
    corners[2] = pointForward + perpendicular * width;
    corners[3] = pointForward - perpendicular * width;
}

template <class T>
inline bool oriented_rect<T>::is_intersect_project(const vec2<T> &v1, const vec2<T> &v2, const vec2<T> &v3,
                                                   const vec2<T> &v4, const vec2<T> &direction, const T min,
                                                   const T max) const {
    const T proj1 = v1 * direction;
    const T proj2 = v2 * direction;
    const T proj3 = v3 * direction;
    const T proj4 = v4 * direction;

    const T min12 = (std::min)(proj1, proj2);
    const T min34 = (std::min)(proj3, proj4);
    const T max12 = (std::max)(proj1, proj2);
    const T max34 = (std::max)(proj3, proj4);

    return !((std::min)(min12, min34) >= max || (std::max)(max12, max34) <= min);
}

template <class T> inline bool oriented_rect<T>::is_intersected(const segment2<T> &segment) const {
    const auto d = segment.p2 - segment.p1;
    if (d == zero2_v<T>)
        return is_point_inside(segment.p1);
    if (!is_intersect_project(corner0() - segment.p1, corner1() - segment.p1, corner2() - segment.p1,
                              corner3() - segment.p1, vec2<T>(-d.y, d.x), T{}, T{}))
        return false;
    const auto a = segment.p1 - center, b = segment.p2 - center;
    const auto ax = a * direction, bx = b * direction, ay = a * perpendicular, by = b * perpendicular;
    return (std::max)(ax, bx) > -length_back && (std::min)(ax, bx) < length_ahead &&
           (std::max)(ay, by) > -width && (std::min)(ay, by) < width;
}

template <class T> inline bool oriented_rect<T>::is_intersected(const oriented_rect<T> &rect) const {
    return is_intersect_project(corner0() - rect.center, corner1() - rect.center, corner2() - rect.center,
                                corner3() - rect.center, rect.direction, -rect.length_back,
                                rect.length_ahead) &&
           is_intersect_project(corner0() - rect.center, corner1() - rect.center, corner2() - rect.center,
                                corner3() - rect.center, rect.perpendicular, -rect.width, rect.width) &&
           is_intersect_project(rect.corner0() - center, rect.corner1() - center, rect.corner2() - center,
                                rect.corner3() - center, direction, -length_back, length_ahead) &&
           is_intersect_project(rect.corner0() - center, rect.corner1() - center, rect.corner2() - center,
                                rect.corner3() - center, perpendicular, -width, width);
}

template <class T> inline bool oriented_rect<T>::is_point_inside(const vec2<T> &point) const {
    const vec2<T> center((corner0().x + corner1().x + corner2().x + corner3().x) / 4,
                         (corner0().y + corner1().y + corner2().y + corner3().y) / 4);
    const int rightSign = geometry_detail::sign(triangle_area2(corner0(), corner1(), center));

    if (rightSign == 0)
        return length_squared(point - center) < T::from_string("0.001");

    return geometry_detail::sign(triangle_area2(corner0(), corner1(), point)) == rightSign &&
           geometry_detail::sign(triangle_area2(corner1(), corner2(), point)) == rightSign &&
           geometry_detail::sign(triangle_area2(corner2(), corner3(), point)) == rightSign &&
           geometry_detail::sign(triangle_area2(corner3(), corner0(), point)) == rightSign;
}

template <class T>
inline bool oriented_rect<T>::is_intersect_circle(const vec2<T> &circleCenter, const T r) const {
    if (is_point_inside(circleCenter))
        return true;

    const vec2<T> vNewCenter = circleCenter - center;
    const vec2<T> vLocalCoordCenter(vNewCenter.x * direction.x + vNewCenter.y * direction.y,
                                    -vNewCenter.x * direction.y + vNewCenter.y * direction.x);

    T fDist = 0;

    if (vLocalCoordCenter.x < -length_back)
        fDist += length_squared(vLocalCoordCenter.x - (-length_back));
    else if (vLocalCoordCenter.x > length_ahead)
        fDist += length_squared(vLocalCoordCenter.x - length_ahead);

    if (vLocalCoordCenter.y < -width)
        fDist += length_squared(vLocalCoordCenter.y - (-width));
    else if (vLocalCoordCenter.y > width)
        fDist += length_squared(vLocalCoordCenter.y - width);

    return fDist <= length_squared(r);
}

template <class T>
inline bool oriented_rect<T>::is_intersect_triangle(const vec2<T> &vTr1, const vec2<T> &vTr2,
                                                    const vec2<T> &vTr3) const {
    if (is_point_inside(vTr1) || is_point_inside(vTr2) || is_point_inside(vTr3))
        return true;
    if (is_point_inside_triangle(vTr1, vTr2, vTr3, corner0()) ||
        is_point_inside_triangle(vTr1, vTr2, vTr3, corner1()) ||
        is_point_inside_triangle(vTr1, vTr2, vTr3, corner2()) ||
        is_point_inside_triangle(vTr1, vTr2, vTr3, corner3()))
        return true;

    segment2<T> segment1(vTr1, vTr2);
    if (is_intersected(segment1))
        return true;

    segment2<T> edge2(vTr1, vTr3);
    if (is_intersected(edge2))
        return true;

    segment2<T> segment3(vTr2, vTr3);
    if (is_intersected(segment3))
        return true;

    return false;
}

template <class T> inline int oriented_rect<T>::get_side(std::uint16_t dirFromRectCenter) const {

    std::uint16_t diff = static_cast<std::uint16_t>(dirFromRectCenter - direction_by_vector(direction));

    if (diff <= 8192)
        return front;
    else if (diff > 8192 && diff <= 24576)
        return left;
    else if (diff > 24576 && diff <= 40960)
        return back;
    else if (diff > 40960 && diff <= 57344)
        return right;
    else
        return front;
}

template <class T> inline int oriented_rect<T>::get_side(const vec2<T> &point) const {
    return get_side(direction_by_vector(point - center));
}

template <class T> inline void oriented_rect<T>::compress(const T fFactor) {
    length_ahead *= fFactor;
    length_back *= fFactor;
    width *= fFactor;

    init_rect(center, direction, length_ahead, length_back, width);
}

template <class T> inline T projected_distance(const oriented_rect<T> rect1, const oriented_rect<T> rect2) {
    vec2<T> direction(rect1.center - rect2.center);
    const T dist = length(direction);
    normalize(&direction);

    const T f1_1 = (rect1.corner0() - rect1.center) * direction + dist;
    const T f1_2 = (rect1.corner1() - rect1.center) * direction + dist;
    const T f1_3 = (rect1.corner2() - rect1.center) * direction + dist;
    const T f1_4 = (rect1.corner3() - rect1.center) * direction + dist;

    const T f2_1 = (rect2.corner0() - rect2.center) * direction;
    const T f2_2 = (rect2.corner1() - rect2.center) * direction;
    const T f2_3 = (rect2.corner2() - rect2.center) * direction;
    const T f2_4 = (rect2.corner3() - rect2.center) * direction;

    const T segm1Min = (std::min)((std::min)(f1_1, f1_2), (std::min)(f1_3, f1_4));
    const T segm1Max = (std::max)((std::max)(f1_1, f1_2), (std::max)(f1_3, f1_4));

    const T segm2Min = (std::min)((std::min)(f2_1, f2_2), (std::min)(f2_3, f2_4));
    const T segm2Max = (std::max)((std::max)(f2_1, f2_2), (std::max)(f2_3, f2_4));

    if (segm1Max < segm2Min || segm2Max < segm1Min)
        return (std::min)(length(segm1Max - segm2Min), length(segm2Max - segm1Min));
    else
        return 0;
}

template <class T>
inline bool intersect_ray_rect(vec2<T> *result, const vec2<T> &origin, const vec2<T> &direction,
                               const oriented_rect<T> &rect) {
    if (result)
        *result = {};
    if (direction == zero2_v<T>)
        return false;
    bool found = false;
    T nearest{};
    vec2<T> hit{};
    for (std::size_t i = 0; i < 4; ++i) {
        const auto a = rect.corners[i], b = rect.corners[(i + 1) % 4];
        const auto edge = b - a, offset = a - origin;
        const auto denominator = cross(direction, edge);
        if (denominator != 0) {
            const auto t = cross(offset, edge) / denominator;
            const auto u = cross(offset, direction) / denominator;
            if (t >= 0 && u >= 0 && u <= 1 && (!found || t < nearest)) {
                found = true;
                nearest = t;
                hit = origin + direction * t;
            }
        } else if (cross(offset, direction) == 0) {
            const auto component =
                fxp::abs(direction.x) >= fxp::abs(direction.y) ? std::size_t{0} : std::size_t{1};
            const auto t0 = (a[component] - origin[component]) / direction[component];
            const auto t1 = (b[component] - origin[component]) / direction[component];
            if ((std::max)(t0, t1) >= 0) {
                const auto t = (std::max)(T{}, (std::min)(t0, t1));
                if (!found || t < nearest) {
                    found = true;
                    nearest = t;
                    hit = origin + direction * t;
                }
            }
        }
    }
    if (found && result)
        *result = hit;
    return found;
}

template <class T> inline std::uint16_t visible_angle(const vec2<T> &point, const oriented_rect<T> rect) {
    std::uint16_t wAngle1 = direction_by_vector(rect.corner0() - point);
    std::uint16_t wAngle2 = direction_by_vector(rect.corner1() - point);
    std::uint16_t wAngle3 = direction_by_vector(rect.corner2() - point);
    std::uint16_t wAngle4 = direction_by_vector(rect.corner3() - point);

    std::uint16_t diff1 = direction_difference(wAngle2, wAngle1);
    std::uint16_t diff2 = direction_difference(wAngle3, wAngle1);
    std::uint16_t diff3 = direction_difference(wAngle4, wAngle1);
    std::uint16_t diff4 = direction_difference(wAngle3, wAngle2);
    std::uint16_t diff5 = direction_difference(wAngle4, wAngle2);
    std::uint16_t diff6 = direction_difference(wAngle4, wAngle3);

    return (std::max)((std::max)((std::max)(diff1, diff2), (std::max)(diff3, diff4)),
                      (std::max)(diff5, diff6));
}

template <class A, class B, std::enable_if_t<!std::is_same_v<A, B>, int> = 0>
auto operator*(const mat4<A> &a, const mat4<B> &b) {
    using R = detail::promoted<A, B>;
    return mat4<R>(a) * mat4<R>(b);
}
template <class A, class B, std::enable_if_t<!std::is_same_v<A, B>, int> = 0>
auto operator*(const quat<A> &a, const quat<B> &b) {
    using R = detail::promoted<A, B>;
    return quat<R>(a) * quat<R>(b);
}
template <class A, class B, std::enable_if_t<!std::is_same_v<A, B>, int> = 0>
auto operator/(const quat<A> &a, const quat<B> &b) {
    using R = detail::promoted<A, B>;
    return quat<R>(a) / quat<R>(b);
}
} // namespace fxp
