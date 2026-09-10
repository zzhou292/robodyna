#include "InputChecks.h"
#include <algorithm>
#include <set>

namespace crash::output::full_shell::source::detail {
namespace {
template<class T> std::vector<T> Values(const CanonicalData& d, const char* name) {
    const auto& a = FindArray(d, name);
    return arrays::Decode<T>(a.descriptor, a.bytes,
        {d.limits.file_bytes, std::max(d.limits.nodes, d.limits.parents), 64});
}
void Unique(std::vector<std::uint64_t> ids) {
    std::sort(ids.begin(), ids.end());
    Require(ids.empty() || (ids.front() && std::adjacent_find(ids.begin(), ids.end()) == ids.end()),
        "Duplicate or zero canonical source ID");
}
std::string IdHash(const CanonicalData& d, const std::vector<std::uint64_t>& ids) {
    return Sha256(arrays::Encode({arrays::Scalar::UInt64, ids.size(), 1, {}}, ids.data(), ids.size(),
        {d.limits.file_bytes, std::max(d.limits.nodes, d.limits.parents), 64}));
}
} // namespace
void CheckCanonicalGeometry(const CanonicalData& d) {
    const auto nodes = Values<std::uint64_t>(d, "node_ids");
    Unique(nodes);
    std::vector<unsigned char> retained_node(nodes.size(), 0);
    std::vector<std::uint64_t> retained_ids, excluded_ids;
    retained_ids.reserve(d.retained_shells);
    excluded_ids.reserve(d.excluded_shells);
    std::set<std::uint64_t> retained_parts, excluded_parts;
    std::size_t retained_q4 = 0, excluded_q4 = 0;
    for (const auto* family : {"shells", "solids", "beams"}) {
        const std::string prefix = family;
        const auto records = Values<std::uint64_t>(d, (prefix + "_records").c_str());
        const auto indices = Values<std::uint32_t>(d, (prefix + "_node_indices").c_str());
        const auto width = FindArray(d, (prefix + "_records").c_str()).descriptor.layout.columns;
        const auto node_width = FindArray(d, (prefix + "_node_indices").c_str()).descriptor.layout.columns;
        std::vector<std::uint64_t> ids;
        ids.reserve(records.size() / width);
        for (std::size_t row = 0; row < records.size() / width; ++row) {
            const auto* record = records.data() + row * width;
            const auto* index = indices.data() + row * node_width;
            ids.push_back(record[0]);
            FindPart(d, record[1]);
            for (std::size_t j = 0; j < node_width; ++j)
                Require(index[j] < nodes.size() && nodes[index[j]] == record[2 + j],
                    "Canonical connectivity differs from source node IDs");
            if (prefix != "shells") continue;
            const bool quad = record[4] != record[5];
            Require(FindPart(d, record[1]).shell_section && record[2] != record[3] &&
                record[2] != record[4] && record[3] != record[4] &&
                (!quad || (record[5] != record[2] && record[5] != record[3])),
                "Unsupported original shell connectivity/section association");
            const bool retained = std::binary_search(d.selected_parts.begin(), d.selected_parts.end(), record[1]);
            const bool excluded = std::binary_search(d.excluded_parts.begin(), d.excluded_parts.end(), record[1]);
            Require(retained != excluded, "Original shell is neither retained nor explicitly omitted");
            (retained ? retained_ids : excluded_ids).push_back(record[0]);
            (retained ? retained_parts : excluded_parts).insert(record[1]);
            if (quad) ++(retained ? retained_q4 : excluded_q4);
            if (retained) for (unsigned j = 0; j < 4; ++j) retained_node[index[j]] = 1;
        }
        Unique(std::move(ids));
    }
    std::vector<std::uint64_t> selected_nodes;
    selected_nodes.reserve(d.retained_nodes);
    for (std::size_t i = 0; i < nodes.size(); ++i) if (retained_node[i]) selected_nodes.push_back(nodes[i]);
    std::sort(selected_nodes.begin(), selected_nodes.end());
    Require(retained_ids.size() == d.retained_shells && excluded_ids.size() == d.excluded_shells &&
        retained_q4 == d.retained_q4 && excluded_q4 == d.excluded_q4 &&
        selected_nodes.size() == d.retained_nodes &&
        std::vector<std::uint64_t>(retained_parts.begin(), retained_parts.end()) == d.selected_parts &&
        std::vector<std::uint64_t>(excluded_parts.begin(), excluded_parts.end()) == d.excluded_parts,
        "Complete retained/omitted source coverage differs");
    Require(IdHash(d, retained_ids) == d.retained_shell_ids_sha256 &&
        IdHash(d, excluded_ids) == d.excluded_shell_ids_sha256 &&
        IdHash(d, selected_nodes) == d.selected_node_ids_sha256,
        "Original retained/omitted source ID digests differ");
}
} // namespace crash::output::full_shell::source::detail
