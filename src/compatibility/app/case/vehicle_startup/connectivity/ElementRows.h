#pragma once
#include "Relations.h"
#include "lib_src/assembly/NodalNodeDomain.h"
#include "output/ArtifactIO.h"

namespace crash::cases::vehicle_startup::connectivity::detail {
// Input is the retained typed reference's original source slots. In particular,
// repeated LAW44 slots are preserved and beam orientation N3 is not a support.
template<class Input>
void AppendElementInput(Relations& output, Kind kind, std::size_t row, const Input& input,
                        tl::util::ConstView<tl::fea::NodalDomainNode> domain,
                        const std::size_t* slots) {
    const auto count = Width(kind);
    output::Require(IsElement(kind) && kind != Kind::Type25 && count &&
        count <= std::size(input.source_node_id) && slots,
        "Connectivity typed element support width differs");
    for (std::size_t slot = 0; slot < count; ++slot)
        output::Require(slots[slot] < domain.size() &&
            domain[slots[slot]].source_id == input.source_node_id[slot],
            "Connectivity element source/domain slot identity differs");
    output.Append(kind,Role::Constitutive,input.source_element_id,input.source_part_id,row,slots,count);
}
} // namespace crash::cases::vehicle_startup::connectivity::detail
