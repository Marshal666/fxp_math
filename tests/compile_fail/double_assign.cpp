#include <fxp/fxp.hpp>
int main() { fxp::fine value; value = 1.23; return value.raw_value() == 0; }
