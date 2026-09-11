#include "Internal.h"

namespace crash::cases::vehicle_startup::connectivity::detail {
namespace {
void CheckRigidSkin(const physical_model::VehiclePhysicalModel& physical,
                    const ReferenceRow& row, const std::size_t* nodes, std::size_t count) {
    const auto& binding = physical.rigid_assembly();
    const auto& topology = *binding.parts()->topology();
    const tl::fea::rigid::PartTopologyPart* part = nullptr;
    for (std::size_t index = 0; index < topology.part_count(); ++index)
        if (topology.parts()[index].source_part_id == row.part_id) part = &topology.parts()[index];
    Require(part && part->root_index == row.rigid_root_index && row.rigid_root_index < topology.root_count(),
            "Connectivity rigid skin has no exact original PART/root association");
    const auto& group = binding.groups()[row.rigid_root_index];
    Require(group.source_kind == tl::fea::RigidBindingSourceKind::Part,
            "Connectivity rigid skin root is not a PART group");
    for (std::size_t slot = 0; slot < count; ++slot) {
        const auto* member = binding.FindMember(nodes[slot]);
        Require(member, "Connectivity rigid skin slot is outside its physical PART");
        const auto index = static_cast<std::size_t>(member-binding.members().data());
        Require(index >= group.member_offset && index-group.member_offset < group.member_count,
                "Connectivity rigid skin slot belongs to a different root");
    }
}
}
void VisitShells(const VehiclePhysicalAttachments& source, Relations& output) {
    const auto& physical = source.physical();
    const auto& shells = physical.shell_source().shells();
    const auto& references = physical.shell_source().references();
    const auto& map = *physical.coefficients().shells();
    std::array<std::size_t,3> visited{};
    for (std::size_t source_row = 0; source_row < references.rows().size(); ++source_row) {
        const auto& row = references.rows()[source_row];
        Require(row.status == ReferenceStatus::Success, "Connectivity shell reference is unresolved");
        std::size_t nodes[4]{};
        Kind kind;
        std::size_t count = 0;
        const auto append = [&](const auto& local, std::uint64_t id) {
            Require(id == row.element_id, "Connectivity source/native shell EID differs");
            count = local.size();
            for (std::size_t slot = 0; slot < count; ++slot) nodes[slot] = map.owner_index(local[slot]);
        };
        if (row.family == ReferenceFamily::Qeph) {
            kind = Kind::Qeph;
            Require(row.reference_index == visited[0]++, "Connectivity QEPH source order changed");
            append(shells.qeph_nodes(row.reference_index),shells.qeph_source_id(row.reference_index));
        } else if (row.family == ReferenceFamily::T3) {
            kind = Kind::T3;
            Require(row.reference_index == visited[1]++, "Connectivity T3 source order changed");
            append(shells.t3_nodes(row.reference_index),shells.t3_source_id(row.reference_index));
        } else {
            Require(row.family == ReferenceFamily::Qbat, "Connectivity shell family is unavailable");
            kind = Kind::Qbat;
            Require(row.reference_index == visited[2]++, "Connectivity QBAT source order changed");
            append(shells.qbat_nodes(row.reference_index),shells.qbat_source_id(row.reference_index));
        }
        const bool rigid = row.role == modelio::vehicle::SourceShellRole::OriginalRigidPart;
        Require(rigid || row.role == modelio::vehicle::SourceShellRole::ConstitutiveShell,
                "Connectivity shell source role is unknown");
        if (rigid) CheckRigidSkin(physical,row,nodes,count);
        output.Append(kind,rigid ? Role::RigidSkin : Role::Constitutive,
                      row.element_id,row.part_id,source_row,nodes,count,rigid ? row.rigid_root_index : SIZE_MAX);
    }
    Require(visited[0] == shells.qeph_count() && visited[1] == shells.t3_count() &&
        visited[2] == shells.qbat_count(), "Connectivity omitted native shell parents");
}
} // namespace crash::cases::vehicle_startup::connectivity::detail
