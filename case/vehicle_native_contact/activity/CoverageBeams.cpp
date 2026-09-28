#include "Coverage.h"
namespace crash::cases::vehicle_native_contact::activity::coverage {
Population Beams(const detail::SourceInputs& in,const Canonical& canonical) {
    const auto& model=in.owner.execution_source().mechanical();const auto& scope=model.embedding().source();
    const auto& type13=scope.type13_source().data();const auto* beam=scope.structural_beam_source();
    output::Require(beam&&type13.beams.size()==model.beams().connection_count()&&
        beam->data().rows.size()==model.structural_beams().parents().size(),"Executed beam support roster is incomplete");
    const auto& array=output::full_shell::source::FindArray(canonical,"beams_records");
    output::Require(array.descriptor.layout.columns==10,"Canonical beam record width differs");
    const auto records=output::arrays::Decode<std::uint64_t>(array.descriptor,array.bytes);
    std::vector<std::uint8_t> selected(records.size()/10);
    for(std::size_t i=0;i<type13.beams.size();++i) {
        const auto& row=type13.beams[i];std::uint64_t nodes[2];
        for(unsigned k=0;k<2;++k) {
            tl::fea::type13::EndpointContribution endpoint;output::Require(model.beams().Endpoint(i,k,endpoint),"TYPE13 support endpoint is unavailable");
            output::Require(endpoint.source_element_id==row.id,"TYPE13 source order differs");nodes[k]=endpoint.source_node_id;
        }
        CheckRecord(records,10,row.canonical_index,row.id,nodes,2,selected);
    }
    for(std::size_t i=0;i<beam->data().rows.size();++i) {
        const auto& row=beam->data().rows[i];std::uint64_t nodes[2];
        for(unsigned k=0;k<2;++k) {
            tl::fea::beam18::EndpointContribution endpoint;output::Require(model.structural_beams().Endpoint(i,k,endpoint),"Beam18 support endpoint is unavailable");
            output::Require(endpoint.source_element_id==row.element_id,"Beam18 source order differs");nodes[k]=endpoint.source_node_id;
        }
        CheckRecord(records,10,row.canonical_row,row.element_id,nodes,2,selected);
    }
    // N3 is original orientation evidence, not native beam/spring node support.
    return Finish(records,10,2,selected,*in.owner.physical().domain());
}
}
