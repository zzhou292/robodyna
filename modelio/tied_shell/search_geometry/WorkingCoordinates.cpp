#include "Internal.h"
#include "modelio/source_assembly/AuxiliarySourceCards.h"
#include <algorithm>
#include <cstring>

namespace crash::modelio::tied_shell::search_detail {
namespace {
bool SameBits(double a, double b) { return std::memcmp(&a, &b, sizeof(a)) == 0; }
std::optional<double> Field(const std::string& row, unsigned offset, unsigned width) {
    auto text = offset < row.size() ? row.substr(offset, width) : std::string{};
    // Same explicit Fortran exponent spelling accepted by the owning original
    // geometry importer. Generic declaration parsing remains unchanged.
    std::replace(text.begin(), text.end(), 'D', 'E');
    std::replace(text.begin(), text.end(), 'd', 'e');
    return vehicle::detail::SourceScalar(text, 0, width);
}
}
void WorkingCoordinates(const source::CanonicalData& source, const Topology& topology,
        const std::string& member, SearchGeometryData& d, SearchGeometryLimits) {
    const auto positions = Decode<double>(source, "node_positions");
    const auto lines = Decode<std::uint32_t>(source, "node_source_lines");
    const auto masks = Decode<std::uint16_t>(source, "node_blank_masks");
    const auto codes = Decode<std::int32_t>(source, "node_codes");
    const auto nn = topology.nodes.size();
    Require(positions.size() == 3*nn && lines.size() == nn && masks.size() == nn && codes.size() == 2*nn,
            "Tied search canonical node field extent changed");
    std::vector<std::pair<std::uint32_t, std::uint32_t>> requests;
    requests.reserve(d.canonical_nodes.size());
    for (std::size_t i = 0; i < d.canonical_nodes.size(); ++i)
        requests.emplace_back(lines.at(d.canonical_nodes[i]), i);
    std::sort(requests.begin(), requests.end());
    d.working_positions.resize(requests.size());
    std::size_t cursor = 0, number = 1, next = 0;
    std::string keyword;
    while (cursor < member.size() && next < requests.size()) {
        const auto end = member.find('\n', cursor);
        const auto stop = end == std::string::npos ? member.size() : end;
        Require(stop-cursor <= 4096, "Tied search original line exceeds bounded width");
        auto text = member.substr(cursor, stop-cursor);
        const auto comment = text.find('$');
        if (comment != std::string::npos) text.resize(comment);
        text = auxiliary::Trim(std::move(text));
        const auto first = text.find_first_not_of(" \t");
        if (first != std::string::npos && text[first] == '*') keyword = text.substr(first);
        if (number == requests[next].first) {
            const auto row = requests[next].second;
            const auto node = d.canonical_nodes[row];
            Require(keyword == "*NODE" && auxiliary::Id(text, 0, 8) == topology.nodes[node] &&
                    auxiliary::BlankTail(text, 72), "Tied original node card identity/format changed");
            unsigned mask = 0;
            for (unsigned axis = 0; axis < 3; ++axis) {
                const auto scalar = Field(text, 8+16*axis, 16);
                if (!scalar) mask |= 1u << (axis+1);
                const double value = scalar.value_or(0.0);
                const double si = value*d.working_length_to_m;
                Require(SameBits(si, positions[3*node+axis]),
                        "Tied source working coordinate differs from canonical SI bits");
                d.working_positions[row][axis] = value;
                d.source_roundtrip_changed_components += !SameBits(si/d.working_length_to_m, value);
            }
            for (unsigned axis = 0; axis < 2; ++axis) {
                const auto scalar = Field(text, 56+8*axis, 8);
                if (!scalar) mask |= 1u << (axis+4);
                Require(scalar.value_or(0) == codes[2*node+axis], "Tied original node codes changed");
            }
            Require(mask == masks[node], "Tied original node blank mask changed");
            ++next;
            Require(next == requests.size() || requests[next].first > number,
                    "Duplicate tied original node line association");
        }
        Require(next == requests.size() || requests[next].first > number,
                "Tied original node line association is invalid");
        cursor = end == std::string::npos ? member.size() : end+1;
        ++number;
    }
    Require(next == requests.size(), "Tied source node card coverage is incomplete");
}
} // namespace crash::modelio::tied_shell::search_detail
