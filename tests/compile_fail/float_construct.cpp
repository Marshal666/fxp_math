#include <fxp/fxp.hpp>
int main() { fxp::real value = 1.23; return value.raw_value() == 0; }
