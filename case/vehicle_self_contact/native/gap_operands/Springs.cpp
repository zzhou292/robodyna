#include "Internal.h"
namespace crash::cases::vehicle_self_contact::native::gap_operands::detail {
void Springs(const source::CorrectedNodalSource& corrected,Packed& out) {
    const auto& seed=corrected.pre_correction();
    std::uint64_t previous=0;
    for(const auto& row:seed.contributors()) {
        Family family;int type;
        switch(row.kind) {
        case nodal_seed::ContributorKind::Type13:family=Family::Type13;type=13;++out.counts.type13;break;
        case nodal_seed::ContributorKind::Type25:family=Family::Type25;type=25;++out.counts.type25;break;
        case nodal_seed::ContributorKind::Type45:family=Family::Type45;type=45;++out.counts.type45;break;
        default:continue;
        }
        Require(row.native_id>previous&&row.slots==2,"Gap native physical SPRING order/extent differs");
        previous=row.native_id;
        values::Spring spring;spring.native_element_id=row.native_id;spring.property_type=type;
        spring.part_contact_thickness=0.; // Complete original/generated-part absence proof.
        for(unsigned k=0;k<2;++k) {
            Require(row.nodes[k]<seed.counts().nodes,"Gap spring node outside shared physical domain");
            spring.nodes[k]=row.nodes[k];
        }
        out.bindings.push_back({family,row.original_id,row.native_id,row.part_id,row.source_index,out.springs.size()});
        out.springs.push_back(spring);
        ++out.proof.skipped_springs;
    }
    Require(out.counts.type13==seed.counts().type13&&out.counts.type25==seed.counts().type25&&
        out.counts.type45==seed.counts().type45,"Gap physical spring family coverage incomplete");
    out.counts.namespace_only_springs=seed.counts().namespace_only_springs;
}
}
