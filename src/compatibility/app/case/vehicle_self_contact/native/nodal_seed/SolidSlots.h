#pragma once
#include "Internal.h"

namespace crash::cases::vehicle_self_contact::native::nodal_seed::detail {

// SPMD_MSIN sees post-INITIA connectivity. Penta's untouched raw slots4/8
// retain original declared source slots0/3 even when S6ZCOOR3 reverses the
// six active slots. Coating's earlier reader packet is a different type/phase.
template<class Parent>
std::array<std::uint32_t, 8> PostInitiaSolidSlots(
    const modelio::solid_source::Row& source, const Parent& parent,
    const tl::fea::NodalNodeDomain& domain, unsigned count) {
    const auto& reference = parent.reference;
    const auto& input = reference.input();
    Require((count == 6 || count == 8) && reference.prepared() &&
        input.source_element_id == source.element_id && input.source_part_id == source.part_id,
        "Contact seed solid source/reference identity differs");
    std::array<std::uint32_t, 8> declared{};
    std::array<std::uint32_t, 8> native{};
    unsigned visited = 0;
    for (unsigned slot = 0; slot < count; ++slot) {
        const unsigned raw = count == 6 ? source.six_to_raw[slot] : slot;
        Require(raw < 8 && input.source_node_id[slot] == source.raw_node_ids[raw],
            "Contact seed solid source slot differs");
        const auto node = parent.domain_nodes[slot];
        Require(node < domain.node_count() && node <= UINT32_MAX &&
            domain.nodes()[node].source_id == input.source_node_id[slot],
            "Contact seed solid physical node differs");
        declared[slot] = static_cast<std::uint32_t>(node);
        const unsigned previous = reference.source_slot(slot);
        Require(previous < count && !(visited & (1u << previous)),
            "Contact seed solid mechanics permutation is invalid");
        visited |= 1u << previous;
    }
    if (count == 8) {
        for (unsigned slot = 0; slot < 8; ++slot)
            native[slot] = declared[reference.source_slot(slot)];
    } else {
        constexpr unsigned active[6]{0, 1, 2, 4, 5, 6};
        for (unsigned slot = 0; slot < 6; ++slot)
            native[active[slot]] = declared[reference.source_slot(slot)];
        native[3] = declared[0];
        native[7] = declared[3];
    }
    return native;
}
} // namespace crash::cases::vehicle_self_contact::native::nodal_seed::detail
