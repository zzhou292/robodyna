#include "Internal.h"
#include "modelio/source_assembly/AuxiliarySourceCards.h"
#include <algorithm>
#include <map>
#include <set>
namespace crash::modelio::vehicle::rigid_part::detail {
void Connections(SourceData& out,Limits limits) {
    namespace cards=tied_shell::detail;
    std::map<std::uint64_t,std::size_t> sets,parts;
    for (std::size_t i=0;i<out.bodies.size();++i)
        Require(parts.emplace(out.bodies[i].source_part_id,i).second,"Duplicate rigid PART identity");
    for (std::size_t i=0;i<out.sources.size();++i) {
        const auto& s=out.sources[i];
        if (s.block.keyword.rfind("*SET_NODE_",0)!=0) continue;
        const bool title=s.block.keyword.size()>=6 && s.block.keyword.substr(s.block.keyword.size()-6)=="_TITLE";
        Require(s.cards.size()>std::size_t(title),"Missing source node-set header");
        Require(sets.emplace(cards::CardId(s.cards[title].second,0),i).second,"Duplicate source node-set identity");
    }
    const auto nodes=[&](std::uint64_t id) {
        const auto found=sets.find(id);
        Require(found!=sets.end(),"Missing rigid source node set");
        const auto& set=out.sources[found->second];
        const bool title=set.block.keyword=="*SET_NODE_LIST_TITLE";
        Require(title || set.block.keyword=="*SET_NODE_LIST","Unsupported rigid node-set operator");
        for (unsigned col=1;col<8;++col)
            Require(!vehicle::detail::SourceScalar(set.cards[title].second,col),"Unsupported rigid node-set header");
        tied_shell::Limits list_limits;
        list_limits.group_members=limits.members;
        return std::make_pair(found->second,cards::ListIds(set,title,list_limits));
    };
    std::set<std::uint64_t> plain_ids;
    std::size_t groups=0,extras=0;
    for (std::size_t i=0;i<out.sources.size();++i) {
        const auto& s=out.sources[i];
        const auto& key=s.block.keyword;
        if (key.rfind("*CONSTRAINED_EXTRA_NODES",0)==0) {
            Require(key=="*CONSTRAINED_EXTRA_NODES_SET" && s.cards.size()==1,"Unsupported rigid extra-node operator");
            const auto& row=s.cards[0].second;
            Require(assembly::reader::auxiliary::BlankTail(row,20),"Unsupported extra-node option");
            const auto found=parts.find(cards::CardId(row,0));
            Require(found!=parts.end(),"Extra-node PART is outside rigid source profile");
            auto& body=out.bodies[found->second];
            Require(body.extra_source==SIZE_MAX,"Repeated rigid extra-node attachment");
            body.node_set_id=cards::CardId(row,1);
            auto set=nodes(body.node_set_id);
            body.node_set_source=set.first;body.extra_nodes=std::move(set.second);body.extra_source=i;
            ++extras;
        } else if (key=="*CONSTRAINED_RIGID_BODIES") {
            for (const auto& card:s.cards) {
                Require(assembly::reader::auxiliary::BlankTail(card.second,20),"Unsupported rigid merge option");
                out.merges.push_back({cards::CardId(card.second,0),cards::CardId(card.second,1)});
            }
        } else if (key.rfind("*CONSTRAINED_NODAL_RIGID_BODY",0)==0) {
            const bool title=key=="*CONSTRAINED_NODAL_RIGID_BODY_TITLE";
            Require(title || key=="*CONSTRAINED_NODAL_RIGID_BODY","Unsupported plain rigid source keyword");
            Require(s.cards.size()==1+std::size_t(title),"Unexpected plain rigid card count");
            const auto& card=s.cards[title].second;
            Require(plain_ids.insert(cards::CardId(card,0)).second,"Duplicate plain rigid source identity");
            auto set=nodes(cards::CardId(card,2));
            Require(set.second.size()<=limits.members-out.plain_rigid_members.size(),"Plain rigid members exceed cap");
            out.plain_rigid_members.insert(out.plain_rigid_members.end(),set.second.begin(),set.second.end());
            ++groups;
        }
    }
    Require(groups==759 && extras==20 && out.merges.size()==2 && out.plain_rigid_members.size()==7539,
            "Original rigid connection inventory changed");
    Require(out.merges[0].parent_part_id==2000387 && out.merges[0].child_part_id==2000414 &&
            out.merges[1].parent_part_id==2000397 && out.merges[1].child_part_id==2000399,
            "Original rigid merge order changed");
}
}
