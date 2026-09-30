#include "Internal.h"

namespace crash::cases::vehicle_startup::connectivity::detail {
void VisitConstraints(const VehiclePhysicalAttachments& source, Relations& output) {
    const auto& physical = source.physical();
    const auto& rigid = physical.rigid_assembly();
    // This fixed scratch is bounded by the existing immutable rigid binding's
    // per-group admission; no member Cartesian product is built.
    std::array<std::size_t,1024> nodes{};
    for (std::size_t row = 0; row < rigid.groups().size(); ++row) {
        const auto& group = rigid.groups()[row];
        Require(group.member_count >= 2 && group.member_count <= nodes.size() &&
            group.member_offset <= rigid.members().size() &&
            group.member_count <= rigid.members().size()-group.member_offset,
            "Connectivity rigid member range is invalid");
        for (std::size_t slot = 0; slot < group.member_count; ++slot)
            nodes[slot] = rigid.members()[group.member_offset+slot].domain_node;
        const bool part = group.source_kind == tl::fea::RigidBindingSourceKind::Part;
        Require(part || group.source_kind == tl::fea::RigidBindingSourceKind::NodalGroup,
                "Connectivity rigid source kind is unknown");
        output.Append(part ? Kind::PartRoot : Kind::PlainGroup,Role::Constraint,
                      group.source_id,part ? group.source_id : 0,row,nodes.data(),group.member_count);
    }
    const auto rows = source.attachments().model().rows();
    for (std::size_t row = 0; row < rows.count; ++row) {
        const auto& cin = rows.data[row];
        nodes[0] = cin.secondary_domain_node;
        for (unsigned slot = 0; slot < 4; ++slot) nodes[slot+1] = cin.master_domain_nodes[slot];
        output.Append(Kind::Cin,Role::Constraint,cin.master_source.element_id,
                      cin.master_source.part_id,row,nodes.data(),5);
    }
}
} // namespace crash::cases::vehicle_startup::connectivity::detail
