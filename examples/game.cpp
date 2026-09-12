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

    real one_third_real = 1_r / 3_r;
    real one_real_ft = one_third_real * 3_r;
    std::cout << "one third real: " << one_third_real << ", one real ft: " << one_real_ft << "\n";  // one third real: 0.3333282470703125, one real ft: 0.9999847412109375

    fine one_third_fine = 1_fine / 3_fine;
    fine one_fine_ft = one_third_fine * 3_fine;
    std::cout << "one third fine: " << one_third_fine << ", one fine ft: " << one_fine_ft << ", as real: " << (real)one_fine_ft << "\n";  // one third fine: 0.33333333325572311878204345703125, one fine ft: 0.99999999976716935634613037109375, as real: 1
}
