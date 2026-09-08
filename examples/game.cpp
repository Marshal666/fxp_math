#include <fxp/fxp.hpp>
#include <iostream>

int main() {
    using fxp::real;
    using fxp::fine;
    using namespace fxp::literals;
    real position = 0;
    const real velocity = 1.2_r;
    const real dt = 1_r / 60;
    for (int frame = 0; frame < 600; ++frame) position += velocity * dt;
    fine precise = 4.00002345_fine;
    auto mixed = position + precise;
    std::cout << "position=" << position << " raw=" << position.raw_value() << '\n'
              << "mixed=" << mixed << " sin=" << fxp::sin(position) << '\n';
    const auto bytes = fxp::to_ieee754_binary32_bytes(position, fxp::byte_order::little_endian);
    std::cout << "binary32 round trip=" << fxp::from_ieee754_binary32_bytes<real>(bytes, fxp::byte_order::little_endian) << '\n';
}
