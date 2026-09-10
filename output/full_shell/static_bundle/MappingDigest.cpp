#include "MappingArrays.h"

namespace crash::output::full_shell::source {
namespace {
void AppendU64(std::string& bytes, std::uint64_t value) {
    for (unsigned j = 0; j < 8; ++j) bytes.push_back(static_cast<char>((value >> (8 * j)) & 255));
}
void AppendText(std::string& bytes, const std::string& value) {
    AppendU64(bytes, value.size());
    bytes.append(value);
}
} // namespace
std::string MappingDigest(const std::array<NamedArray, 8>& entries, arrays::Limits cap) {
    detail::CheckMappingArrays(entries, cap);
    std::string input;
    AppendText(input, "robo_dyna.full_shell_source_mapping.digest.v1");
    AppendU64(input, entries.size());
    for (const auto& a : entries) {
        const auto& layout = a.descriptor.layout;
        AppendText(input, a.name);
        AppendText(input, arrays::Dtype(layout.scalar));
        AppendU64(input, layout.rows);
        AppendU64(input, layout.columns);
        AppendU64(input, layout.fields.size());
        for (const auto& field : layout.fields) AppendText(input, field);
        AppendU64(input, a.descriptor.bytes);
        AppendText(input, a.descriptor.sha256);
    }
    return Sha256(input);
}
} // namespace crash::output::full_shell::source
