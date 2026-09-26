#include "SupportQuery.h"
#include <algorithm>
#include <cmath>
namespace crash::cases::vehicle_self_contact::native::main_coefficients::detail {
std::size_t SupportQueryBytes(std::size_t nodes, std::size_t shells) {
    if (!nodes || nodes > 524288 || shells > 524288)
        Reject(Status::ResourceLimit, "Mixed shell support index extent exceeds source profile");
    // Persistent offsets/rows and temporary cursors, each vector charged at
    // twice its upper element count. All arithmetic is bounded by hard counts.
    return 2*(2*(nodes+1)+4*shells)*sizeof(std::uint32_t)+
        4*shells*sizeof(std::size_t)+sizeof(SupportQueryIndex)+4096;
}
SupportQueryIndex PrepareSupportQueries(const coated::Inputs& input, const Packed& packed,
        std::size_t byte_cap) {
    const auto required = SupportQueryBytes(input.nodes.size(), input.shells.size());
    if (!byte_cap || byte_cap > (std::size_t{64}<<20) || required > byte_cap)
        Reject(Status::ResourceLimit, "Mixed shell support index exceeds admitted byte cap");
    Require(packed.shells.size() == input.shells.size() && packed.keys.size() == input.shells.size(),
        "Mixed support index lacks complete physical shell operands");
    SupportQueryIndex result;
    result.source_shells = input.shells.data();
    result.shell_count = input.shells.size();
    result.node_count = input.nodes.size();
    result.quad_offsets.assign(input.nodes.size()+1, 0);
    std::size_t count = 0;
    for (std::size_t i = 0; i < input.shells.size(); ++i) {
        const auto& face = input.shells[i].primary;
        const bool triangle = face.layout == n::ShellLayout::Triangle3;
        Require(triangle || face.layout == n::ShellLayout::Quad4, "Mixed support physical shell layout is unavailable");
        Require(!triangle || face.nodes[2] == face.nodes[3], "Mixed support T3 duplicate slot differs");
        Require(packed.shells[i].part < packed.parts.size(), "Mixed support shell part operand is unavailable");
        const auto& value = packed.parts[packed.shells[i].part].coefficient;
        Require(std::isfinite(value.property_thickness) && value.property_thickness > 0 &&
            std::isfinite(value.young) && value.young > 0,
            "Mixed support query requires the existing positive ordinary property profile");
        for (unsigned k = 0; k < 4; ++k) Require(face.nodes[k] < input.nodes.size(), "Mixed support shell node is outside domain");
        const unsigned arity = triangle ? 3 : 4;
        for (unsigned k = 0; k < arity; ++k)
            for (unsigned j = 0; j < k; ++j)
                Require(face.nodes[k] != face.nodes[j], "Mixed support physical shell repeats an active node");
        if (!triangle) {
            for (const auto node : face.nodes) ++result.quad_offsets[node+1];
            count += 4;
        }
    }
    for (std::size_t i = 1; i < result.quad_offsets.size(); ++i)
        result.quad_offsets[i] += result.quad_offsets[i-1];
    result.quad_rows.resize(count);
    auto cursor = result.quad_offsets;
    // BUILD_CNEL's corner-major occurrence structure. The supplied physical
    // row order remains representative; the selection certificate resolves
    // genuine corner/material-group precedence instead of claiming this order.
    for (unsigned k = 0; k < 4; ++k)
        for (std::size_t i = 0; i < input.shells.size(); ++i) {
            const auto& face = input.shells[i].primary;
            if (face.layout == n::ShellLayout::Quad4)
                result.quad_rows[cursor[face.nodes[k]]++] = std::uint32_t(i);
        }
    if (result.quad_offsets.capacity() > 2*(input.nodes.size()+1) ||
        cursor.capacity() > 2*(input.nodes.size()+1) || result.quad_rows.capacity() > 8*input.shells.size())
        Reject(Status::ResourceLimit, "Mixed support index allocation exceeds reservation");
    return result;
}
namespace {
FaceKey QueryKey(const s::Main& main, bool triangle) {
    FaceKey key;
    key.arity = triangle ? 3 : 4;
    std::copy_n(main.nodes, 4, key.nodes);
    std::sort(key.nodes, key.nodes+key.arity);
    if (triangle) key.nodes[3] = UINT32_MAX;
    return key;
}
SupportSelection Exact(const Packed& packed, const FaceKey& key) {
    const auto start = std::lower_bound(packed.keys.begin(), packed.keys.end(), key, KeyLess);
    SupportSelection result;
    double thickness = 0, young = 0;
    for (auto at = start; at != packed.keys.end() && SameKey(*at, key); ++at)
        AccumulateSupport(packed, at->physical, result, thickness, young);
    return result;
}
}
SupportSelection QuerySupport(const coated::Inputs& input, const Packed& packed, const SupportQueryIndex& index,
        const s::Main& main, bool grouping_context) {
    Require(index.source_shells == input.shells.data() && index.shell_count == input.shells.size() &&
        index.node_count == input.nodes.size() && index.quad_offsets.size() == input.nodes.size()+1 &&
        packed.shells.size() == input.shells.size(), "Mixed support query has a foreign or incomplete index");
    const bool triangle = main.nodes[2] == main.nodes[3];
    const unsigned arity = triangle ? 3 : 4;
    for (unsigned k = 0; k < 4; ++k) Require(main.nodes[k] < input.nodes.size(), "Mixed support query node is outside domain");
    for (unsigned k = 0; k < arity; ++k)
        for (unsigned j = 0; j < k; ++j) Require(main.nodes[k] != main.nodes[j], "Mixed support query repeats an active node");
    auto result = Exact(packed, QueryKey(main, triangle));
    // I25GAPM tests NELTG first. A Q4 value changing the final local DXM does
    // not clear NELTG, so it cannot displace an already-found T3 owner.
    if (!result.winners.empty() || !triangle)
        return ResolveSupportTies(input, packed, main, std::move(result), triangle, grouping_context);
    const auto first = index.quad_offsets[main.nodes[0]];
    const auto last = index.quad_offsets[main.nodes[0]+1];
    Require(first <= last && last <= index.quad_rows.size(), "Mixed support Q4 incidence range differs");
    double thickness = 0, young = 0;
    for (auto at = first; at < last; ++at) {
        const auto row = index.quad_rows[at];
        Require(row < input.shells.size(), "Mixed support incidence row is outside physical source");
        const auto& face = input.shells[row].primary;
        Require(face.layout == n::ShellLayout::Quad4, "Mixed support subset index has a non-Q4 row");
        bool contains = true;
        for (unsigned k = 0; k < arity; ++k)
            contains = contains && std::find(std::begin(face.nodes), std::end(face.nodes), main.nodes[k]) != std::end(face.nodes);
        if (contains) AccumulateSupport(packed, row, result, thickness, young);
    }
    return ResolveSupportTies(input, packed, main, std::move(result), false, grouping_context);
}
}
