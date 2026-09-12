#include "Composition.h"
#include "case/vehicle_startup/physical_model/VehiclePhysicalModel.h"
namespace crash::cases::vehicle_wall {
output::physical_run::WallComposition CaptureComposition(
    const vehicle_startup::physical_model::VehiclePhysicalModel& model) {
    using output::Require;
    namespace fe=tl::fea;
    using DomainPolicy=modelio::physical_domain::Policy;
    using SolidPolicy=modelio::solid_source::Policy;
    using Profile=output::physical_run::CompositionProfile;
    const auto& domain=model.source_domain();
    const auto& source=domain.source().solid_source().data();
    const auto& solids=model.solids();const auto& ledger=model.coefficients();
    const auto& rigid=model.rigid_assembly();const auto& scope=ledger.scope();
    const bool supports=domain.policy()==DomainPolicy::RetainedShellAssembliesVehicleSupportsV5;
    const bool extended=domain.policy()==DomainPolicy::RetainedShellAssembliesExtendedSolidsV4;
    Require(supports || extended || domain.policy()==DomainPolicy::RetainedShellAssembliesV1,
        "Wall composition requires a named physical source domain");
    Require(source.policy==(supports?SolidPolicy::OriginalVehicleSupportsV5:
            extended?SolidPolicy::OriginalExtendedSolidsV4:SolidPolicy::OriginalAdhesive18RubberHephS6zV1) &&
        solids.profile()==((supports || extended)?fe::solids::ModelProfile::ExtendedLaw44Law90:fe::solids::ModelProfile::OriginalThreeFamilies) &&
        ledger.order()==(supports?fe::CoefficientOrder::PreparedSI_Q_T_B_Type25_Type13_ElementMass_Solid18_24_6z_Law44_Law90_Beam18_V5:
            extended?fe::CoefficientOrder::PreparedSI_Q_T_B_Type25_Type13_ElementMass_Solid18_24_6z_Law44_Law90_V4:
            fe::CoefficientOrder::PreparedSI_Q_T_B_Type25_Type13_ElementMass_Solid18_24_6z_V3),
        "Wall composition domain, solid source, model and ledger profiles disagree");
    Require(ledger.prepared() && ledger.domain()->SharesStorage(domain.domain()) &&
        solids.domain()->SharesStorage(domain.domain()) && rigid.coefficients()->Matches(ledger) &&
        !scope.uncovered_nodes && scope.covered_nodes==domain.domain().node_count(),
        "Wall composition requires the actual complete physical ledger");
    output::physical_run::WallComposition c;
    c.profile=supports?Profile::VehicleSupportsV5:extended?Profile::ExtendedSolidsV4:Profile::RetainedV1;
    c.physical_nodes=domain.domain().node_count();c.solid_parts=source.parts.size();
    c.solid_parents={solids.solid18().size(),solids.solid24().size(),solids.solid6z().size(),
        solids.solid18_law44().size(),solids.solid18_law90().size()};
    const std::array<std::uint64_t,5> ledger_counts{scope.solid18_parents,scope.solid24_parents,
        scope.solid6z_parents,scope.solid18_law44_parents,scope.solid18_law90_parents};
    Require(c.solid_parents==ledger_counts,"Wall composition solid model and ledger counts differ");
    std::uint64_t parents=0;for(auto count:c.solid_parents)parents+=count;
    Require(parents==source.rows.size(),"Wall composition omits selected solid source parents");
    const auto* beams=model.structural_beams();
    const auto* beam_source=domain.source().structural_beam_source();
    const auto* beam_coefficients=ledger.beam18();
    Require(bool(beams)==supports && bool(beam_source)==supports && bool(beam_coefficients)==supports,
        "Wall composition structural beam authorities differ");
    if(supports) {
        Require(beams->prepared() && beams->profile()==fe::beam18::ModelProfile::CircularFourPointLaw44V1 &&
            beams->domain()->SharesStorage(domain.domain()) &&
            beam_coefficients->model()->SharesStorage(*beams) &&
            beam_source->data().policy==modelio::beam18::Policy::OriginalCircularFourPointLaw44V1 &&
            &beam_source->canonical().data()==&domain.source().solid_source().canonical().data(),
            "Wall composition structural beam source/model/ledger identity differs");
        c.structural_beam_parents=beams->parents().size();
        c.structural_beam_parts=beam_source->data().parts.size();
        Require(c.structural_beam_parents==beam_source->data().rows.size() &&
                c.structural_beam_parents==scope.beam18_parents &&
                beam_coefficients->record_count()==2*c.structural_beam_parents,
                "Wall composition structural beam endpoint census differs");
        for(std::size_t i=0;i<beams->parents().size();++i) {
            const auto& input=beams->parents()[i].reference.input();
            const auto& row=beam_source->data().rows[i];
            Require(input.source_element_id==row.element_id && input.source_part_id==row.part_id,
                "Wall composition structural beam source order differs");
        }
    } else Require(scope.beam18_parents==0 && scope.occurrences.beam18==0,
        "Legacy wall composition contains undeclared structural beams");
    c.point_mass_records=scope.element_mass_records;
    c.part_roots=rigid.parts()->roots().size();c.rigid_groups=rigid.groups().size();
    c.rigid_members=rigid.members().size();
    const auto groups=domain.counts();
    c.plain_complete=groups.complete_groups;c.plain_restricted=groups.restricted_groups;
    c.plain_omitted=groups.omitted_groups;
    Require(c.point_mass_records==groups.retained_point_masses,
        "Wall composition selected point cards and ledger differ");
    c.initial_mass_kg=ledger.totals().mass;c.point_mass_kg=ledger.totals().element_mass;
    // Same bounded field validation as the independent archive reader.
    output::physical_run::WallCompositionDocument(c);
    return c;
}
} // namespace crash::cases::vehicle_wall
