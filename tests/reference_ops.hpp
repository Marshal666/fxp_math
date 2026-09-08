#pragma once
#include <fxp/fxp.hpp>
#include <fstream>
#include <sstream>
#include <vector>

namespace reference {
struct row { unsigned format; std::string operation, a, b, result; int auxiliary; std::uint64_t tolerance; };
inline std::vector<row> read(const std::string& path) {
    std::ifstream in(path);
    if (!in) throw std::runtime_error("cannot open reference file: " + path);
    std::vector<row> rows;
    std::string line;
    while (std::getline(in, line)) {
        if (line.empty() || line[0] == '#') continue;
        std::istringstream stream(line);
        std::string fields[7];
        for (auto& field : fields) if (!std::getline(stream, field, ',')) throw std::runtime_error("invalid reference row");
        rows.push_back({static_cast<unsigned>(std::stoul(fields[0])), fields[1], fields[2], fields[3], fields[4],
            std::stoi(fields[5]), static_cast<std::uint64_t>(std::stoull(fields[6]))});
    }
    if (rows.empty()) throw std::runtime_error("empty reference file: " + path);
    return rows;
}
struct result { std::string value; int auxiliary = 0; };
template<class T> result evaluate(const row& r) {
    try {
        const auto& op = r.operation;
        if (op == "parse") return {std::to_string(T::from_string(r.a).raw_value())};
        if (op == "from32") return {std::to_string(fxp::from_ieee754_binary32_bits<T>(static_cast<std::uint32_t>(std::stoull(r.a))).raw_value())};
        if (op == "from64") return {std::to_string(fxp::from_ieee754_binary64_bits<T>(static_cast<std::uint64_t>(std::stoull(r.a))).raw_value())};
        const auto a = T::from_raw(static_cast<std::int64_t>(std::stoll(r.a)));
        const auto b = T::from_raw(static_cast<std::int64_t>(std::stoll(r.b)));
        if (op == "to32") return {std::to_string(fxp::to_ieee754_binary32_bits(a))};
        if (op == "to64") return {std::to_string(fxp::to_ieee754_binary64_bits(a))};
        if (op == "convert") {
            using Other = fxp::fixed<T::fractional_bits == 16 ? 32 : 16>;
            return {std::to_string(Other(a).raw_value())};
        }
        if (op == "integer") return {std::to_string(a.template to_integer<>())};
        T out;
        int aux = 0;
        if (op == "add") out = a + b;
        else if (op == "sub") out = a - b;
        else if (op == "mul") out = a * b;
        else if (op == "div") out = a / b;
        else if (op == "fmod") out = fxp::fmod(a, b);
        else if (op == "remainder") out = fxp::remainder(a, b);
        else if (op == "remquo") out = fxp::remquo(a, b, &aux);
        else if (op == "copysign") out = fxp::copysign(a, b);
        else if (op == "abs") out = fxp::abs(a);
        else if (op == "floor") out = fxp::floor(a);
        else if (op == "ceil") out = fxp::ceil(a);
        else if (op == "trunc") out = fxp::trunc(a);
        else if (op == "round") out = fxp::round(a);
        else if (op == "nearbyint") out = fxp::nearbyint(a);
        else if (op == "sin") out = fxp::sin(a);
        else if (op == "cos") out = fxp::cos(a);
        else if (op == "tan") out = fxp::tan(a);
        else if (op == "asin") out = fxp::asin(a);
        else if (op == "acos") out = fxp::acos(a);
        else if (op == "atan") out = fxp::atan(a);
        else if (op == "atan2") out = fxp::atan2(a, b);
        else if (op == "exp") out = fxp::exp(a);
        else if (op == "exp2") out = fxp::exp2(a);
        else if (op == "expm1") out = fxp::expm1(a);
        else if (op == "log") out = fxp::log(a);
        else if (op == "log2") out = fxp::log2(a);
        else if (op == "log10") out = fxp::log10(a);
        else if (op == "log1p") out = fxp::log1p(a);
        else if (op == "sqrt") out = fxp::sqrt(a);
        else if (op == "cbrt") out = fxp::cbrt(a);
        else if (op == "hypot") out = fxp::hypot(a, b);
        else if (op == "pow") out = fxp::pow(a, b);
        else if (op == "powi") out = fxp::pow(a, static_cast<std::int64_t>(std::stoll(r.b)));
        else if (op == "ldexp") out = fxp::ldexp(a, std::stoi(r.b));
        else if (op == "roundtrip") out = T::from_string(a.to_string());
        else throw std::runtime_error("unknown reference operation: " + op);
        return {std::to_string(out.raw_value()), aux};
    } catch (const std::domain_error&) { return {"domain"}; }
      catch (const std::invalid_argument&) { return {"invalid"}; }
}
inline result evaluate(const row& r) {
    if (r.format == 16) return evaluate<fxp::real>(r);
    if (r.format == 32) return evaluate<fxp::fine>(r);
    throw std::runtime_error("invalid reference format");
}
} // namespace reference
