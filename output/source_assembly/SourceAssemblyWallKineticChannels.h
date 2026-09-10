#pragma once
#include "SourceAssemblyKineticSchema.h"
#include "WallFieldValues.h"
#include "lib_src/constraints/NodalRigidObservationTypes.h"

namespace crash::output::assembly::wall_fields {
inline Value KineticMember(Document& d,const tl::fea::rigid::MemberKineticChannels& c) {
    return Values(d,{c.translation,c.native_rotation,c.physical_rotation,c.added_rotation,c.total,c.inertia_partition_residual});
}
// Preserve the established stored-midpoint field order and exact scalar values.
// The caller supplies the distinct time/phase contract for its observation.
inline void KineticChannels(Document& d,const tl::fea::rigid::MemberKineticChannels& ordinary,
    const tl::fea::rigid::MemberKineticChannels& members,const tl::fea::rigid::AggregateKineticChannels& g,
    double native,double effective) {
    String(d,"member_columns",KineticMemberColumns);
    Put(d,"ordinary_native_nodes",KineticMember(d,ordinary));Put(d,"grouped_native_members",KineticMember(d,members));
    String(d,"aggregate_columns",KineticAggregateColumns);
    Put(d,"aggregate_groups",Values(d,{g.translation,g.rotation,g.total,g.structural_translation,g.primary_translation,
        g.member_orbital_rotation,g.native_member_rotation,g.physical_member_rotation,g.added_member_rotation,
        g.primary_parallel_axis_rotation,g.primary_isotropic_rotation,g.principal_correction_rotation,g.decomposition_residual,g.decomposition_roundoff_budget}));
    Number(d,"native_total_J",native);Number(d,"effective_total_J",effective);
}
} // namespace crash::output::assembly::wall_fields
