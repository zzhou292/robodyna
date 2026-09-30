#include "NativeCoordinates.h"
#include "SourceFields.h"
#include "output/BoundedArrayIO.h"
#include "modelio/source_assembly/AuxiliarySourceCards.h"
#include <algorithm>
#include <cstring>

namespace crash::modelio::source_nodes {
using output::Require;
namespace auxiliary = modelio::assembly::reader::auxiliary;
namespace {
bool SameBits(double a, double b) { return std::memcmp(&a, &b, sizeof(a)) == 0; }
std::optional<double> Field(const std::string& row, unsigned offset, unsigned width) {
    auto text = offset < row.size() ? row.substr(offset, width) : std::string{};
    // Same explicit Fortran exponent spelling accepted by the owning original
    // geometry importer. Generic declaration parsing remains unchanged.
    std::replace(text.begin(), text.end(), 'D', 'E');
    std::replace(text.begin(), text.end(), 'd', 'e');
    return modelio::assembly::reader::SourceScalar(text, 0, width);
}
}
template<class T> std::vector<T> Decode(const source::CanonicalData& source, const char* name) {
    const auto& array = source::FindArray(source, name);
    return output::arrays::Decode<T>(array.descriptor, array.bytes,
        {source.limits.file_bytes, std::max(source.limits.nodes, source.limits.parents), 64});
}
std::size_t NativeCoordinateBytes(const source::CanonicalData& source, std::size_t count) {
    Require(count <= source.canonical_nodes && count <= 1048576, "Native node request exceeds source domain");
    std::size_t total = sizeof(NativeCoordinates), largest = 0;
    for (const auto* name : {"node_positions", "node_source_lines", "node_blank_masks", "node_codes"}) {
        const auto bytes = source::FindArray(source, name).descriptor.bytes;
        Require(bytes <= SIZE_MAX - total, "Native coordinate decode forecast overflows");
        total += bytes;
        largest = std::max(largest, bytes);
    }
    constexpr auto width = sizeof(std::array<double, 3>) + sizeof(std::pair<std::uint32_t, std::uint32_t>);
    Require(count <= (SIZE_MAX - total) / width, "Native coordinate request forecast overflows");
    total += count * width;
    Require(largest <= SIZE_MAX - total, "Native coordinate temporary forecast overflows");
    return total + largest;
}
NativeCoordinates ReadNativeCoordinates(const source::CanonicalData& source,
        const std::vector<std::uint64_t>& canonical_ids,
        const std::vector<std::uint32_t>& canonical_nodes, const std::string& member) {
    (void)NativeCoordinateBytes(source, canonical_nodes.size());
    source::CheckUnits(source.inputs.units);
    const double length = source.inputs.units.length_to_m;
    NativeCoordinates result;
    const auto positions = Decode<double>(source, "node_positions");
    const auto lines = Decode<std::uint32_t>(source, "node_source_lines");
    const auto masks = Decode<std::uint16_t>(source, "node_blank_masks");
    const auto codes = Decode<std::int32_t>(source, "node_codes");
    const auto nn = canonical_ids.size();
    Require(nn == source.canonical_nodes, "Native source node ID extent changed");
    Require(positions.size() == 3*nn && lines.size() == nn && masks.size() == nn && codes.size() == 2*nn,
            "Native source canonical node field extent changed");
    std::vector<std::pair<std::uint32_t, std::uint32_t>> requests;
    requests.reserve(canonical_nodes.size());
    for (std::size_t i = 0; i < canonical_nodes.size(); ++i)
        requests.emplace_back(lines.at(canonical_nodes[i]), i);
    std::sort(requests.begin(), requests.end());
    for (std::size_t i = 1; i < requests.size(); ++i)
        Require(requests[i-1].first != requests[i].first, "Repeated native source node request");
    result.positions.resize(requests.size());
    std::size_t cursor = 0, number = 1, next = 0;
    std::string keyword;
    while (cursor < member.size() && next < requests.size()) {
        const auto end = member.find('\n', cursor);
        const auto stop = end == std::string::npos ? member.size() : end;
        Require(stop-cursor <= 4096, "Native source original line exceeds bounded width");
        auto text = member.substr(cursor, stop-cursor);
        const auto comment = text.find('$');
        if (comment != std::string::npos) text.resize(comment);
        text = auxiliary::Trim(std::move(text));
        const auto first = text.find_first_not_of(" \t");
        if (first != std::string::npos && text[first] == '*') keyword = text.substr(first);
        if (number == requests[next].first) {
            const auto row = requests[next].second;
            const auto node = canonical_nodes[row];
            Require(keyword == "*NODE" && auxiliary::Id(text, 0, 8) == canonical_ids[node] &&
                    auxiliary::BlankTail(text, 72), "Native source node card identity/format changed");
            unsigned mask = 0;
            for (unsigned axis = 0; axis < 3; ++axis) {
                const auto scalar = Field(text, 8+16*axis, 16);
                if (!scalar) mask |= 1u << (axis+1);
                const double value = scalar.value_or(0.0);
                const double si = value*length;
                Require(SameBits(si, positions[3*node+axis]),
                        "Native source working coordinate differs from canonical SI bits");
                result.positions[row][axis] = value;
                result.roundtrip_changed_components += !SameBits(si/length, value);
            }
            for (unsigned axis = 0; axis < 2; ++axis) {
                const auto scalar = Field(text, 56+8*axis, 8);
                if (!scalar) mask |= 1u << (axis+4);
                Require(scalar.value_or(0) == codes[2*node+axis], "Native source node codes changed");
            }
            Require(mask == masks[node], "Native source node blank mask changed");
            ++next;
            Require(next == requests.size() || requests[next].first > number,
                    "Duplicate native source node line association");
        }
        Require(next == requests.size() || requests[next].first > number,
                "Native source node line association is invalid");
        cursor = end == std::string::npos ? member.size() : end+1;
        ++number;
    }
    Require(next == requests.size(), "Native source node card coverage is incomplete");
    return result;
}
} // namespace crash::modelio::source_nodes
