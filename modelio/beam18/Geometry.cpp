#include "Internal.h"
#include <algorithm>
#include <map>
#include <set>
namespace crash::modelio::beam18::detail {
void ReadGeometry(const source::CanonicalData& source,const std::string&,Data& data,Limits limits) {
    const auto records=Decode<std::uint64_t>(source,"beams_records");
    const auto indices=Decode<std::uint32_t>(source,"beams_node_indices");
    const auto lines=Decode<std::uint32_t>(source,"beams_source_lines");
    const auto masks=Decode<std::uint16_t>(source,"beams_blank_masks");
    const auto ids=Decode<std::uint64_t>(source,"node_ids");
    Require(records.size()%10==0 && indices.size()==records.size()/10*2 && lines.size()==records.size()/10 &&
        masks.size()==lines.size() && ids.size()==source.canonical_nodes && std::is_sorted(ids.begin(),ids.end()),
        "Beam18 canonical array extents or node ordering changed");
    std::map<std::uint64_t,std::size_t> parts;
    for(std::size_t i=0;i<data.parts.size();++i)parts.emplace(data.parts[i].id,i);
    std::set<std::uint64_t> unique;
    std::set<std::uint32_t> needed,endpoints;
    std::size_t counts[4]{};
    data.original_beams=lines.size();data.rows.reserve(142);
    for(std::size_t e=0;e<lines.size();++e) {
        const auto* record=records.data()+10*e;
        Require(unique.insert(record[0]).second,"Duplicate original beam EID");
        if(!Selected(record[1])) {++data.outside_beams;continue;}
        const auto found=parts.find(record[1]);
        Require(found!=parts.end() && data.rows.size()<limits.parents && masks[e]==480 && lines[e]>0 &&
            record[9]==2,"Selected original beam declaration or capacity changed");
        Row row;row.element_id=record[0];row.part_id=record[1];row.part_index=found->second;
        row.canonical_row=e;row.source_line=lines[e];row.blank_mask=masks[e];
        std::copy_n(record,10,row.raw_record.begin());
        for(unsigned k=5;k<9;++k)Require(record[k]==0,"Unsupported original beam release");
        for(unsigned slot=0;slot<3;++slot) {
            const auto at=std::lower_bound(ids.begin(),ids.end(),record[2+slot]);
            Require(at!=ids.end() && *at==record[2+slot],"Original beam endpoint/orientation node missing");
            row.nodes[slot]=at-ids.begin();needed.insert(row.nodes[slot]);
            if(slot<2){Require(indices[2*e+slot]==row.nodes[slot],"Canonical beam endpoint mapping changed");endpoints.insert(row.nodes[slot]);}
        }
        Require(row.nodes[0]!=row.nodes[1],"Repeated structural beam endpoints");
        ++counts[row.part_index];data.rows.push_back(std::move(row));
    }
    Require(needed.size()<=limits.nodes && data.parts.size()==4 && counts[0]==36 && counts[1]==36 &&
        counts[2]==35 && counts[3]==35,"Original beam18 part/node census changed");
    data.canonical_endpoints.assign(endpoints.begin(),endpoints.end());
    const auto node_lines=Decode<std::uint32_t>(source,"node_source_lines");
    const auto node_masks=Decode<std::uint16_t>(source,"node_blank_masks");
    const auto positions=Decode<double>(source,"node_positions");
    Require(node_lines.size()==ids.size() && node_masks.size()==ids.size() && positions.size()==3*ids.size(),
        "Beam18 canonical node array extent changed");
    data.nodes.reserve(needed.size());
    std::map<std::uint32_t,std::uint32_t> local;
    for(auto index:needed) {
        Node n;n.id=ids[index];n.canonical_index=index;n.source_line=node_lines[index];n.blank_mask=node_masks[index];
        n.position_m={positions[3*index],positions[3*index+1],positions[3*index+2]};
        local.emplace(index,data.nodes.size());data.nodes.push_back(std::move(n));
    }
    for(auto& row:data.rows)for(auto& node:row.nodes)node=local.at(node);
}
} // namespace crash::modelio::beam18::detail
