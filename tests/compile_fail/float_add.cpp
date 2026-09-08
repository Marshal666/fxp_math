#include <fxp/fxp.hpp>
int main() { auto value = fxp::real(1) + 1.23f; return value.raw_value() == 0; }
