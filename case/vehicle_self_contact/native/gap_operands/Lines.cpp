#include "Internal.h"
#include "lib_src/elements/beam18/PropertyArea.h"
#include <cmath>
namespace crash::cases::vehicle_self_contact::native::gap_operands::detail {
void Lines(const source::CorrectedNodalSource& corrected,Packed& out) {
    const auto& seed=corrected.pre_correction();
    const auto* model=seed.physical().structural_beams();
    Require(model&&model->domain()&&model->domain()->SharesStorage(seed.physical().source_domain().domain()),
        "Gap beam model has foreign physical authority");
    for(const auto& row:seed.contributors()) {
        if(row.kind!=nodal_seed::ContributorKind::Beam18)continue;
        Require(row.source_index<model->parents().size()&&row.slots==2,"Gap beam source row differs");
        const auto& parent=model->parents()[row.source_index];
        const auto& reference=parent.reference;const auto& input=reference.input();
        Require(input.units==tl::fea::beam18::WorkingUnits::TonneMillimetreSecond ||
            input.units==tl::fea::beam18::WorkingUnits::SI,"Unknown prepared beam working units");
        const n::UnitScale units=input.units==tl::fea::beam18::WorkingUnits::TonneMillimetreSecond
            ? n::UnitScale{.001,1000.,1.}:n::UnitScale{1.,1.,1.};
        Require(reference.prepared()&&input.profile==tl::fea::beam18::Profile::CircularFourPointStoredZero&&
            SameUnits(units,seed.provenance().units)&&input.source_element_id==row.original_id&&
            input.source_part_id==row.part_id,"Gap beam reference/profile/units association differs");
        values::Line line;line.source_element_id=row.original_id;line.part_contact_thickness=0.;
        const auto status=tl::fea::beam18::EvaluateCircularPropertyArea(input.radius,&line.native_area);
        if(status!=tl::fea::beam18::Status::Success)
            Reject(Status::NonfiniteResult,"Pre-PMASS circular beam property area unavailable",row.original_id);
        for(unsigned k=0;k<2;++k) {
            Require(parent.domain_nodes[k]==row.nodes[k]&&row.nodes[k]<seed.counts().nodes,
                "Gap beam endpoints differ from complete contributor source");
            line.nodes[k]=row.nodes[k];
        }
        CheckMaximumTerm(.5*std::sqrt(line.native_area));
        ++out.proof.maximum_terms;
        out.bindings.push_back({Family::Beam18,row.original_id,row.native_id,row.part_id,row.source_index,out.beams.size()});
        out.beams.push_back(line);
    }
    out.counts.beams=out.beams.size();
    Require(out.counts.beams==seed.counts().beams&&out.counts.beams==model->parents().size(),
        "Gap physical beam coverage incomplete");
}
}
