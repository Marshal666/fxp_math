#include "../tests/reference_ops.hpp"
#include <iostream>
int main(int argc, char** argv) {
    if (argc != 3) { std::cerr << "usage: fxp_reference_writer oracle.csv output.csv\n"; return 2; }
    try {
        const auto rows = reference::read(argv[1]);
        std::ofstream out(argv[2]);
        if (!out) throw std::runtime_error("cannot write reference output");
        out << "# fxp_math bit-exact reference v1; format,operation,a,b,result,auxiliary,tolerance\n";
        for (const auto& r : rows) {
            const auto actual = reference::evaluate(r);
            out << r.format << ',' << r.operation << ',' << r.a << ',' << r.b << ',' << actual.value << ',' << actual.auxiliary << ",0\n";
        }
        if (!out) throw std::runtime_error("reference write failed");
    } catch (const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
}
