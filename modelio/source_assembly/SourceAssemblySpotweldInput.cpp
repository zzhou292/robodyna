#include "SourceAssemblySpotweldInput.h"
#include "ResolvedSpotweldProperty.h"
#include "output/ArtifactIO.h"
#include <algorithm>

namespace crash::modelio::assembly {
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
    property_={declaration.generated_property_id,ResolvedSpotweldProperty()};
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
    result.source_units=SpotweldSourceUnits;result.properties=&property_;result.property_count=1;
    result.connections=connections_.empty()?nullptr:connections_.data();result.connection_count=connections_.size();
    return result;
}
} // namespace crash::modelio::assembly
