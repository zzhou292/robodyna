#include "InputChecks.h"
#include <algorithm>
#include <cmath>
#include <string_view>

namespace crash::output::full_shell::source {
void CheckUnits(const Units& u) {
    Require(!u.mass.empty() && !u.length.empty() && !u.time.empty() &&
        u.mass.size() <= 32 && u.length.size() <= 32 && u.time.size() <= 32 &&
        std::isfinite(u.mass_to_kg) && u.mass_to_kg > 0 &&
        std::isfinite(u.length_to_m) && u.length_to_m > 0 &&
        std::isfinite(u.time_to_s) && u.time_to_s > 0, "Invalid source units");
}
bool SameUnits(const Units& a, const Units& b) noexcept {
    return a.mass == b.mass && a.length == b.length && a.time == b.time &&
        Bits(a.mass_to_kg) == Bits(b.mass_to_kg) && Bits(a.length_to_m) == Bits(b.length_to_m) &&
        Bits(a.time_to_s) == Bits(b.time_to_s);
}
const NamedArray& FindArray(const CanonicalData& d, const char* name) {
    for (const auto& a : d.arrays) if (a.name == name) return a;
    throw std::runtime_error("Missing canonical array");
}
const PartDeclaration& FindPart(const CanonicalData& d, std::uint64_t id) {
    const auto it = std::lower_bound(d.parts.begin(), d.parts.end(), id,
        [](const PartDeclaration& p, std::uint64_t x) { return p.part < x; });
    Require(it != d.parts.end() && it->part == id, "Missing source part declaration");
    return *it;
}
namespace detail {
const Value& Field(const Value& v, const char* name) {
    Require(v.IsObject(), "Expected source metadata object");
    const Value* found = nullptr;
    for (auto it = v.MemberBegin(); it != v.MemberEnd(); ++it) {
        if (std::string_view(it->name.GetString(), it->name.GetStringLength()) == name) {
            Require(!found, "Duplicate source metadata field");
            found = &it->value;
        }
    }
    Require(found, "Missing source metadata field");
    return *found;
}
std::string ReadFile(const std::filesystem::path& root, const RecordFile& f, std::size_t cap) {
    Require(f.bytes && f.bytes <= cap, "Source file exceeds declared capacity");
    arrays::CheckHash(f.sha256);
    const auto path = arrays::CheckedPath(root, f.file, true);
    Require(std::filesystem::file_size(path) == f.bytes, "Source file size changed");
    auto bytes = ReadBounded(path, f.bytes);
    Require(bytes.size() == f.bytes && Sha256(bytes) == f.sha256, "Source file identity changed");
    return bytes;
}
void AddBytes(std::size_t& value, std::size_t add, std::size_t cap) {
    Require(value <= cap && add <= cap - value, "Static source byte capacity exceeded");
    value += add;
}
} // namespace detail
} // namespace crash::output::full_shell::source
