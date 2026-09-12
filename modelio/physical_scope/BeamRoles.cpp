#include "Internal.h"
namespace crash::modelio::physical_scope::detail {
void BuildBeamRoles(const beam18::Source& source,const std::vector<SourceId>& nodes,Data& data) {
    const auto& beams=source.data();
    const auto records=Decode<std::uint64_t>(source.canonical().data(),"beams_records");
    Require(records.size()%10==0 && data.node_roles.size()==nodes.size(),"Structural beam role extent differs");
    for(const auto& row:beams.rows) {
        Require(row.canonical_row<records.size()/10,"Structural beam source row exceeds canonical inventory");
        const auto* original=records.data()+10*row.canonical_row;
        Require(original[0]==row.element_id && original[1]==row.part_id,"Structural beam source EID/PID differs");
        for(unsigned slot=0;slot<3;++slot) {
            const auto& node=beams.nodes.at(row.nodes[slot]);
            Require(node.canonical_index<nodes.size() && nodes[node.canonical_index]==node.id &&
                original[slot+2]==node.id,"Structural beam source node/slot differs");
            data.node_roles[node.canonical_index]|=slot<2?Beam18Endpoint:BeamOrientation;
        }
    }
}
} // namespace crash::modelio::physical_scope::detail
