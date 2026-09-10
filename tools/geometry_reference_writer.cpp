#if !defined(FXP_PORTABLE_ONLY) || !defined(FXP_DISABLE_SSE2)
#error "Geometry reference results require the portable scalar backend"
#endif
#include "../tests/geometry_reference.hpp"
#include <fstream>
#include <iostream>

int main(int argc, char **argv) {
    if (argc != 2) {
        std::cerr << "usage: fxp_geometry_reference_writer output.csv\n";
        return 2;
    }
    try {
        const auto rows = geometry_reference::generate();
        std::ofstream out(argv[1]);
        out << "# fxp geometry reference v2 (portable scalar C++); format,case,operation,raw outputs\n";
        for (const auto &row : rows)
            out << row << '\n';
        out.flush();
        if (!out)
            throw std::runtime_error("could not write geometry reference file");
        std::cout << "Wrote " << rows.size() << " geometry reference cases\n";
    } catch (const std::exception &e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
}
