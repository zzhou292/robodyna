#include "SourceAssemblySpotweldInput.h"
#include "output/ArtifactIO.h"
#include <algorithm>

namespace crash::modelio::assembly {
namespace {
constexpr tl::fea::type25::SourceUnits Units{1000.,.001,1.};
tl::fea::type25::Property ResolvedProperty() {
    // Pinned mm_s_Mg row: unitsystemdefaults.cxx88-107; Ileng0 dimensions:
    // prop_p25_spr_axi.cfg328-329,355 and corresponding rotational channels.
    const double mass_unit=Units.mass_to_kg;
    const double inertia_unit=mass_unit*Units.length_to_m*Units.length_to_m;
    const double force_unit=mass_unit*Units.length_to_m/(Units.time_to_s*Units.time_to_s);
    const double moment_unit=force_unit*Units.length_to_m;
    tl::fea::type25::Property p;
    p.mass_kg=.001e-3*mass_unit; p.isotropic_inertia_kg_m2=.01e-3*inertia_unit;
    p.stiffness[0]=p.stiffness[1]=100.e3*mass_unit/(Units.time_to_s*Units.time_to_s);
    p.stiffness[2]=p.stiffness[3]=1000.e3*inertia_unit/(Units.time_to_s*Units.time_to_s);
    // Blank SN/SS/N/M resolve to0 at direct SDI import; HM_READ_PROP25 replaces
    // zero rupture limits with +/-EP30*channel units and zero alpha/beta with
    //1/2. Missing damping is0. These are explicit converter regularizers.
    for(unsigned c=0;c<4;++c) {
        p.failure_positive[c]=1.e30*(c<2?force_unit:moment_unit);
        p.failure_negative[c]=-p.failure_positive[c];
        p.failure_weight[c]=1.; p.failure_exponent[c]=2.;
    }
    return p;
}
}
SourceAssemblySpotweldInput::SourceAssemblySpotweldInput(const SourceAssembly& source,
    std::uint64_t instance,SpotweldDeclaration declaration)
    :source_(source),source_instance_(instance),declaration_(declaration) {
    output::Require(instance&&declaration.generated_property_id&&
        declaration.policy==SpotweldPolicy::OpenRadiossTonneMillimetreSecondDirectImport,
        "Internal spotwelds require explicit source instance, generated property ID and source-unit policy");
    const auto& data=source_.data();
    output::Require(!data.internal_spotwelds.empty(),"Source spotweld input requires complete internal endpoints");
    output::Require(std::none_of(data.sections.begin(),data.sections.end(),[&](const auto& section) {
        return section.id==declaration.generated_property_id;
    }),"Generated TYPE25 property identity overlaps an original shell section");
    property_={declaration.generated_property_id,ResolvedProperty()};
    connections_.reserve(data.internal_spotwelds.size());
    for(const auto& weld:data.internal_spotwelds) {
        // Keep the policy subset explicit if the general reader later expands.
        output::Require(weld.record.cards.size()==2&&weld.record.cards[0].blank_mask==254&&
            weld.record.cards[1].blank_mask==252&&weld.record.external_nodes.empty(),
            "Spotweld policy supports only complete WID/N1/N2-only source records");
        tl::fea::type25::ConnectionInput c; c.source_element_id=weld.record.id;
        for(unsigned e=0;e<2;++e) {
            const auto n=weld.nodes[e];
            output::Require(n<data.nodes.size()&&data.nodes[n].source_id==weld.record.node_ids[e],
                            "Internal spotweld endpoint differs from source node inventory");
            c.global_node[e]=n;c.source_node_id[e]=weld.record.node_ids[e];c.position[e]=data.nodes[n].position_m;
        }
        connections_.push_back(c);
    }
}
tl::fea::type25::ModelInput SourceAssemblySpotweldInput::input() const noexcept {
    tl::fea::type25::ModelInput result;
    result.source_instance_id=source_instance_;result.global_node_count=source_.data().nodes.size();
    result.source_units=Units;result.properties=&property_;result.property_count=1;
    result.connections=connections_.empty()?nullptr:connections_.data();result.connection_count=connections_.size();
    return result;
}
} // namespace crash::modelio::assembly
