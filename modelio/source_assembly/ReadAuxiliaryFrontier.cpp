#include "AuxiliarySourceCards.h"
#include <algorithm>
#include <set>

namespace crash::modelio::assembly::reader {
void ReadAuxiliaryFrontier(const Value& attachments,const Value& frontier,const ReadLimits& limits,Data& data) {
    if(data.schema!=SectionInventorySchema) {
        Require(!attachments.HasMember("auxiliary_frontier"),"Auxiliary frontier requires explicit V3 source scope");return;
    }
    auto& out=data.boundary.auxiliary;const auto& value=Member(attachments,"auxiliary_frontier");
    TextIs(value,"policy","released_external_auxiliary_nodes_v1");
    Flag(value,"mechanics_qualified",false);Flag(value,"selected_owner_nodes_added",false);
    Flag(value,"selected_owner_mass_added",false);
    out.source_node_ids=Ids(Member(value,"source_node_ids"),limits.external_nodes,true);
    std::set<SourceId> missing(data.boundary.external_node_ids.begin(),data.boundary.external_node_ids.end());
    for(const auto& e:Array(frontier,"elements",4096).GetArray())
        for(auto n:Ids(Member(e,"touched_node_ids"),8,true)) {
            Require(std::binary_search(data.boundary.external_node_ids.begin(),data.boundary.external_node_ids.end(),n),
                    "Auxiliary structural incidence names a non-frontier node");missing.erase(n);
        }
    Require(out.source_node_ids==std::vector<SourceId>(missing.begin(),missing.end()),
            "Auxiliary disposition must cover exactly the nonstructural frontier");
    std::vector<AuxiliaryPointMass> masses;std::vector<AuxiliarySphericalJoint> joints;
    std::set<SourceId> mass_nodes,joint_nodes,mass_ids,joint_ids;
    std::size_t prior_line=0;
    for(const auto& encoded:Array(value,"source_blocks",2*limits.external_nodes).GetArray()) {
        auto block=Block(encoded);Require(block.filename=="yaris-coarse-v1l.key"&&block.first_line>prior_line,
            "Auxiliary blocks must retain unique original source order");prior_line=block.first_line;
        const auto cards=auxiliary::RawCards(encoded,block);bool used=false;
        if(block.keyword=="*ELEMENT_MASS") {
            for(std::size_t i=0;i<cards.size();++i) {
                if(cards[i].empty())continue;
                const auto node=auxiliary::Id(cards[i],8,8);if(!missing.count(node))continue;
                const auto eid=auxiliary::Id(cards[i],0,8);const auto mass=auxiliary::Number(cards[i],16,16);
                const auto si=mass*data.units.mass_to_kg;
                Require(mass_ids.insert(eid).second&&mass>0&&std::isfinite(si)&&si>0&&auxiliary::BlankTail(cards[i],32),
                        "Unsupported auxiliary point-mass source options");
                masses.push_back({eid,node,block.first_line,i,mass,si});mass_nodes.insert(node);used=true;
            }
        } else {
            Require(block.keyword=="*CONSTRAINED_JOINT_SPHERICAL_ID"&&cards.size()==2,
                    "Unsupported auxiliary joint source form");
            const auto id=auxiliary::Id(cards[0],0,10);
            const std::array<SourceId,2> nodes{auxiliary::Id(cards[1],0,10),auxiliary::Id(cards[1],10,10)};
            Require(joint_ids.insert(id).second&&nodes[0]!=nodes[1]&&auxiliary::BlankTail(cards[0],10)&&
                auxiliary::BlankTail(cards[1],20),"Unsupported auxiliary spherical joint options");
            for(auto n:nodes)if(missing.count(n)){joint_nodes.insert(n);used=true;}
            joints.push_back({id,nodes,block.first_line});
        }
        Require(used,"Unused auxiliary source block");out.source_blocks.push_back(std::move(block));
    }
    Require(mass_nodes==missing&&joint_nodes==missing,"Auxiliary node lacks complete mass/joint evidence");
    const auto& supplied_masses=Array(value,"point_masses",limits.external_nodes,masses.size());
    Require(supplied_masses.Size()==masses.size(),"Auxiliary point-mass coverage changed");
    for(unsigned i=0;i<supplied_masses.Size();++i) {
        const auto& v=supplied_masses[i];const auto& m=masses[i];
        Require(Unsigned(v,"source_element_id")==m.source_element_id&&Unsigned(v,"source_node_id")==m.source_node_id&&
            Unsigned(v,"source_block_line")==m.source_block_line&&Unsigned(v,"source_card_index")==m.source_card_index,
            "Auxiliary point-mass source association changed");
        Same(Real(v,"supplied_mass_source"),m.supplied_mass_source);Same(Real(v,"supplied_mass_kg"),m.supplied_mass_kg);
    }
    const auto& supplied_joints=Array(value,"spherical_joints",limits.external_nodes,joints.size());
    Require(supplied_joints.Size()==joints.size(),"Auxiliary joint coverage changed");
    for(unsigned i=0;i<supplied_joints.Size();++i) {
        const auto& v=supplied_joints[i];const auto& j=joints[i];const auto nodes=Ids(Member(v,"source_node_ids"),2);
        Require(Unsigned(v,"source_joint_id")==j.source_joint_id&&Unsigned(v,"source_block_line")==j.source_block_line&&
            nodes==std::vector<SourceId>(j.source_node_ids.begin(),j.source_node_ids.end()),
            "Auxiliary joint source association changed");
    }
    out.point_masses=std::move(masses);out.spherical_joints=std::move(joints);
}
} // namespace crash::modelio::assembly::reader
