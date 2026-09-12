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
    const bool extended=domain.policy()==DomainPolicy::RetainedShellAssembliesExtendedSolidsV4;
    Require(extended || domain.policy()==DomainPolicy::RetainedShellAssembliesV1,
        "Wall composition requires a named physical source domain");
    Require(source.policy==(extended?SolidPolicy::OriginalExtendedSolidsV4:SolidPolicy::OriginalAdhesive18RubberHephS6zV1) &&
        solids.profile()==(extended?fe::solids::ModelProfile::ExtendedLaw44Law90:fe::solids::ModelProfile::OriginalThreeFamilies) &&
        ledger.order()==(extended?fe::CoefficientOrder::PreparedSI_Q_T_B_Type25_Type13_ElementMass_Solid18_24_6z_Law44_Law90_V4:
            fe::CoefficientOrder::PreparedSI_Q_T_B_Type25_Type13_ElementMass_Solid18_24_6z_V3),
        "Wall composition domain, solid source, model and ledger profiles disagree");
    Require(ledger.prepared() && ledger.domain()->SharesStorage(domain.domain()) &&
        solids.domain()->SharesStorage(domain.domain()) && rigid.coefficients()->Matches(ledger) &&
        !scope.uncovered_nodes && scope.covered_nodes==domain.domain().node_count(),
        "Wall composition requires the actual complete physical ledger");
    output::physical_run::WallComposition c;
    c.profile=extended?Profile::ExtendedSolidsV4:Profile::RetainedV1;
    c.physical_nodes=domain.domain().node_count();c.solid_parts=source.parts.size();
    c.solid_parents={solids.solid18().size(),solids.solid24().size(),solids.solid6z().size(),
        solids.solid18_law44().size(),solids.solid18_law90().size()};
    const std::array<std::uint64_t,5> ledger_counts{scope.solid18_parents,scope.solid24_parents,
        scope.solid6z_parents,scope.solid18_law44_parents,scope.solid18_law90_parents};
    Require(c.solid_parents==ledger_counts,"Wall composition solid model and ledger counts differ");
    std::uint64_t parents=0;for(auto count:c.solid_parents)parents+=count;
    Require(parents==source.rows.size(),"Wall composition omits selected solid source parents");
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
