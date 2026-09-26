#include "Internal.h"
#include <algorithm>
namespace crash::cases::vehicle_self_contact::native::main_coefficients::detail {
bool KeyLess(const FaceKey& a, const FaceKey& b) noexcept {
    if (a.arity != b.arity)return a.arity<b.arity;
    for (unsigned k = 0; k<4; ++k)if (a.nodes[k] != b.nodes[k])return a.nodes[k]<b.nodes[k];
    return a.physical<b.physical;
}
bool SameKey(const FaceKey& a, const FaceKey& b) noexcept {
    if (a.arity != b.arity)return false;
    for (unsigned k = 0; k<4; ++k)if (a.nodes[k] != b.nodes[k])return false;
    return true;
}
FaceKey Key(const coated::Shell& shell, std::size_t physical) {
    Require(physical <= UINT32_MAX,"Physical shell ordinal exceeds key representation");
    Require(shell.primary.layout == n::ShellLayout::Triangle3 || shell.primary.layout == n::ShellLayout::Quad4,
        "Unknown shell layout in native support key");
    FaceKey key; key.physical = std::uint32_t(physical);
    key.arity = shell.primary.layout == n::ShellLayout::Triangle3?3:4;
    std::copy_n(shell.primary.nodes, 4, key.nodes);
    std::sort(key.nodes, key.nodes+key.arity);
    if (key.arity == 3)key.nodes[3] = UINT32_MAX;
    return key;
}
SupportSelection SelectSupport(const coated::Inputs& input, const Packed& packed, const s::Main& main,
    std::size_t physical, bool grouping_context) {
    auto key = Key(input.shells.at(physical), 0);
    const auto start = std::lower_bound(packed.keys.begin(), packed.keys.end(), key, KeyLess);
    SupportSelection result; double thickness = 0, young = 0;
    for (auto at = start; at != packed.keys.end() && SameKey(*at, key); ++at) {
        const auto i = at->physical;
        const auto& value = packed.parts.at(packed.shells.at(i).part).coefficient;
        // Positive ordinary property1 TYPE25: exact INCOQ3 DX then ST policy.
        if (value.property_thickness>thickness ||
            (value.property_thickness == thickness && value.young>young)) {
            thickness = value.property_thickness; young = value.young; result.winners.clear();
        }
        if (value.property_thickness == thickness && value.young == young)result.winners.push_back(i);
    }
    Require(!result.winners.empty(),"Selected primary has no physical shell support");
    if (result.winners.size() == 1) {
        result.owner = result.winners.front(); result.proof = OwnerProof::UniqueBest; return result;
    }
    const bool triangle = input.shells[physical].primary.layout == n::ShellLayout::Triangle3;
    const auto corner = [&](std::size_t i) {
        const auto& nodes = input.shells[i].primary.nodes;
        for (unsigned k = 0; k<(triangle?3u:4u); ++k)if (nodes[k] == main.nodes[0])return k;
        throw std::runtime_error("Native incidence pivot is absent from matching shell");
    };
    unsigned selected_corner = corner(result.winners[0]);
    for (const auto i:result.winners)selected_corner = triangle?std::max(selected_corner, corner(i)):std::min(selected_corner, corner(i));
    std::vector<std::size_t> same_corner;
    for (const auto i:result.winners)if (corner(i) == selected_corner)same_corner.push_back(i);
    if (same_corner.size() == 1) {
        result.owner = same_corner[0]; result.proof = OwnerProof::NativeCornerOrder; return result;
    }
    if (!grouping_context)return result;
    for (const auto i:same_corner) {
        const auto& part = packed.parts[packed.shells[i].part];
        if (!part.native_material_id_preserved)return result;
        for (const auto j:same_corner)if (i != j) {
            const auto& other = packed.parts[packed.shells[j].part];
            if (part.mid == other.mid || !GroupPrefixEqual(part, other))return result;
        }
    }
    result.owner = same_corner[0];
    for (const auto i:same_corner) {
        const auto mid = packed.parts[packed.shells[i].part].mid;
        const auto best = packed.parts[packed.shells[result.owner].part].mid;
        if (triangle?mid>best:mid<best)result.owner = i;
    }
    result.proof = OwnerProof::NativeMaterialGroupOrder;
    return result;
}
void CheckIdentity(const s::Main& main, const n::NativeExteriorMainGeometryResult& geometry,
    std::size_t primary, std::uint64_t eid) {
    for (unsigned k = 0; k<4; ++k)
        if (geometry.source_corner[k] >= 4 || main.nodes[geometry.source_corner[k]] != main.nodes[k])
            Reject(Status::NeedsPrimaryOrientationPhase,"Native primary permutation is not identity; partners are unchanged",
                primary, eid);
}
} // namespace crash::cases::vehicle_self_contact::native::main_coefficients::detail
