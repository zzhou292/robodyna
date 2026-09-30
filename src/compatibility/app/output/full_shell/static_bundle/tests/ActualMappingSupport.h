#pragma once
#include "../MappingRecords.h"
#include "../MappingArrays.h"
#include "../MappingFields.h"
#include "../../tests/TestSupport.h"
#include <algorithm>
#include <cstdlib>
#include <numeric>

namespace crash::output::full_shell::source::test {
inline SourceInputs ActualInputs() {
    const auto* canonical = std::getenv("ROBO_STATIC_CANONICAL");
    const auto* scope = std::getenv("ROBO_STATIC_SCOPE");
    const auto* member = std::getenv("ROBO_STATIC_MEMBER");
    Require(canonical && scope && member, "Missing explicit original-source fixture");
    SourceInputs in;
    in.canonical_root = canonical;
    in.scope_root = std::filesystem::path(scope).parent_path();
    in.member_root = std::filesystem::path(member).parent_path();
    in.canonical_manifest = {"manifest.json", "c82f1886b8935d69ff7db4c29c700370e3a057579fab80d02664a253bc7af1c8", 2632394};
    in.scope_report = {std::filesystem::path(scope).filename().string(),
        "da85bfffc01f96f962c5e8658a66912e915838f3b376afc3a39d3cf71ae32f1b", 13175122};
    in.source_member = {std::filesystem::path(member).filename().string(),
        "67208317e6c8eb1dd43b80001508915ccaace7bc0a745e1aa5a3b33f394df301", 42846753};
    in.tire_policy = "omit_original_tire_shells";
    in.units = {"t", "mm", "s", 1000, .001, 1};
    return in;
}
inline const CanonicalSource& ActualSource() {
    static const auto source = CanonicalSource::Read(ActualInputs());
    return source;
}
struct ActualOrder {
    explicit ActualOrder(const CanonicalSource& source) {
        const auto& d = source.data();
        const auto& record = FindArray(d, "shells_records");
        const auto& conn = FindArray(d, "shells_node_indices");
        const auto r = arrays::Decode<std::uint64_t>(record.descriptor, record.bytes);
        const auto c = arrays::Decode<std::uint32_t>(conn.descriptor, conn.bytes);
        std::vector<unsigned char> used(d.canonical_nodes, 0);
        for (std::size_t i = 0; i < d.canonical_shells; ++i) {
            if (!std::binary_search(d.selected_parts.begin(), d.selected_parts.end(), r[6 * i + 1])) continue;
            // Opaque fixture-only family token; all native fields unavailable.
            // This is source-shaped formatting evidence, not material admission.
            parents.push_back({static_cast<std::uint32_t>(i), 0x53434f50u,
                static_cast<std::uint32_t>(parents.size()), 0, PlasticField::Unavailable});
            for (unsigned j = 0; j < 4; ++j) used[c[4 * i + j]] = 1;
        }
        for (std::size_t i = 0; i < used.size(); ++i) if (used[i]) nodes.push_back(static_cast<std::uint32_t>(i));
        std::reverse(nodes.begin(), nodes.end());
        std::reverse(parents.begin(), parents.end());
    }
    MappingInput View() const { return {nodes.data(), nodes.size(), parents.data(), parents.size()}; }
    std::vector<std::uint32_t> nodes;
    std::vector<NativeParent> parents;
};
inline const PreparedSourceMapping& ActualMapping() {
    static const auto mapping = [] {
        const auto& source = ActualSource();
        ActualOrder order(source);
        return PreparedSourceMapping::Prepare(source, order.View());
    }();
    return mapping;
}
} // namespace crash::output::full_shell::source::test
