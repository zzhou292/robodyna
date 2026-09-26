#include "Internal.h"
#include <algorithm>

namespace crash::cases::vehicle_self_contact::native::initial_surfaces::detail {
namespace {
ShellKey Key(const std::uint32_t* nodes, unsigned arity) {
    ShellKey key;
    key.arity = arity;
    std::copy_n(nodes, arity, key.nodes.begin());
    std::sort(key.nodes.begin(), key.nodes.begin() + arity);
    if (arity == 3) key.nodes[3] = key.nodes[2];
    return key;
}
bool Less(const ShellKey& a, const ShellKey& b) {
    return a.arity < b.arity || (a.arity == b.arity && a.nodes < b.nodes);
}
bool SameIdentity(const Identity& a, const Identity& b) {
    return a.kind == b.kind && a.element_id == b.element_id && a.part_id == b.part_id &&
        a.canonical_row == b.canonical_row && a.source_line == b.source_line && a.solid_face == b.solid_face;
}
}
Report CertifyMembership(const Packing& input, const values::Snapshot& probe, Certificate& out) {
    // The successful qualified SOLID probe has already admitted all physical
    // Q4/T3 rows and query faces as distinct corners. Equal sorted node sets
    // within the SAME family are therefore exactly native ISHEL==3/4, not a
    // geometric/proximity or cross-family predicate.
    std::vector<ShellKey> keys;
    keys.reserve(input.quads.size() + input.triangles.size());
    const auto add = [&](const auto& rows, unsigned arity) {
        for (const auto& row : rows) {
            auto key = Key(row.nodes, arity);
            key.element = row.element_id;
            key.selected = std::binary_search(input.selected_parts.begin(), input.selected_parts.end(), row.part_id);
            keys.push_back(key);
        }
    };
    add(input.quads, 4);
    add(input.triangles, 3);
    std::sort(keys.begin(), keys.end(), Less);
    Certificate next = out;
    for (std::size_t i = 0; i < probe.face_count; ++i) {
        const auto& face = probe.faces[i];
        output::Require(face.source.kind == values::ParentKind::Solid && face.raw_role == 1,
            "Suppression certificate requires the unsuppressed solid-only probe");
        const auto key = Key(face.nodes, face.nodes[2] == face.nodes[3] ? 3 : 4);
        const auto first = std::lower_bound(keys.begin(), keys.end(), key, Less);
        const auto last = std::upper_bound(first, keys.end(), key, Less);
        ++next.queried_solid_faces;
        next.matching_physical_shells += std::size_t(last - first);
        for (auto at = first; at != last; ++at) {
            if (at->selected != first->selected) {
                Report failure;
                failure.status = Status::NeedsNativeReaderOrder;
                failure.reason = "First matching early shell has order-dependent selected membership";
                failure.solid_element = face.source.element_id;
                failure.solid_face = face.source.solid_face;
                failure.first_candidate_element = first->element;
                failure.conflicting_candidate_element = at->element;
                return failure;
            }
        }
    }
    next.membership_complete = true;
    out = next;
    return {Status::Ready, "Complete physical matching-shell membership is invariant"};
}
Report CertifyConsumerOrder(const std::vector<Face>& faces, Certificate& out, std::vector<OriginGroup>* published) {
    std::vector<OriginGroup> groups;
    if (published) groups.reserve(faces.size()/2);
    Certificate next = out;
    for (std::size_t first = 0; first < faces.size();) {
        std::size_t last = first + 1;
        while (last < faces.size() && faces[last].nodes == faces[first].nodes) ++last;
        if (last < faces.size()) output::Require(faces[first].nodes < faces[last].nodes,
            "CREATE surface result is not sorted by original four-node words");
        if (last - first > 1) {
            ++next.equal_node_key_groups;
            bool different_origin = false;
            for (std::size_t i = first + 1; i < last; ++i) {
                // I25SURFI reads ELEM only for the pre-filter set-to-one union.
                // Equal nodes/role therefore have identical consumed fields,
                // while every source origin remains present and tagged.
                if (faces[first].raw_role != faces[i].raw_role) {
                    Report failure;
                    failure.status = Status::NeedsNativeReaderOrder;
                    failure.reason = "Equal CREATE node keys have distinct unqualified role ordering";
                    failure.first_candidate_element = faces[first].source.element_id;
                    failure.conflicting_candidate_element = faces[i].source.element_id;
                    return failure;
                }
                different_origin = different_origin || !SameIdentity(faces[first].source, faces[i].source);
            }
            if (different_origin) ++next.differing_origin_groups;
            if (published) groups.push_back({first, last-first});
        }
        first = last;
    }
    next.consumed_order_complete = true;
    out = next;
    if (published) published->swap(groups);
    return {Status::Ready, "Consumed node/role order invariant; all raw origins retained"};
}
}
