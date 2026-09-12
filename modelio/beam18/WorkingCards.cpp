#include "Internal.h"
#include "modelio/source_assembly/SourceLines.h"
#include <algorithm>
namespace crash::modelio::beam18::detail {
namespace {
double Scalar(const std::string& text,std::size_t offset,unsigned width,unsigned bit,unsigned mask) {
    const auto field=text.substr(std::min(offset,text.size()),width);
    const bool blank=auxiliary::Trim(field).empty();
    Require(blank==bool(mask&(1u<<bit)),"Original beam18 numerical blank mask changed");
    return blank ? 0 : RequiredScalar(field,0,width);
}
}
void ReadWorkingCards(const source::CanonicalData& source,const std::string& member,Data& data) {
    struct Request{std::uint32_t line,index;bool node;};
    std::vector<Request> requests;requests.reserve(data.nodes.size()+data.rows.size());
    for(std::size_t i=0;i<data.nodes.size();++i)requests.push_back({data.nodes[i].source_line,static_cast<std::uint32_t>(i),true});
    for(std::size_t i=0;i<data.rows.size();++i)requests.push_back({data.rows[i].source_line,static_cast<std::uint32_t>(i),false});
    std::sort(requests.begin(),requests.end(),[](const auto& a,const auto& b){return a.line<b.line;});
    std::vector<std::uint32_t> lines;for(const auto& r:requests)lines.push_back(r.line);
    const auto codes=Decode<std::int32_t>(source,"node_codes");
    Require(codes.size()==2*source.canonical_nodes,"Beam18 motion-code extent changed");
    VisitSourceLines(member,lines,[&](std::size_t i,const std::string& keyword,const std::string& card){
        const auto& request=requests[i];
        if(request.node) {
            auto& node=data.nodes[request.index];
            Require(keyword=="*NODE" && auxiliary::BlankTail(card,72) && auxiliary::Id(card,0,8)==node.id &&
                !(node.blank_mask&1) && node.blank_mask<=63,"Original beam18 NODE identity/format changed");
            double working[3],si[3]{node.position_m.x,node.position_m.y,node.position_m.z};
            for(unsigned k=0;k<3;++k){working[k]=Scalar(card,8+16*k,16,k+1,node.blank_mask);
                Require(output::Bits(working[k]*.001)==output::Bits(si[k]),"Beam18 working/SI coordinate bits differ");}
            for(unsigned k=0;k<2;++k)Require(codes[2*node.canonical_index+k]==0 &&
                Scalar(card,56+8*k,8,k+4,node.blank_mask)==0,"Original beam18 NODE motion code unsupported");
            node.position_working={working[0],working[1],working[2]};node.raw_card=card;
        } else {
            auto& row=data.rows[request.index];
            Require(keyword=="*ELEMENT_BEAM" && auxiliary::BlankTail(card,80),"Original beam18 element keyword/format changed");
            for(unsigned k=0;k<10;++k)Require(Scalar(card,8*k,8,k,row.blank_mask)==double(row.raw_record[k]),
                "Original beam18 raw connectivity/release association changed");
            row.raw_card=card;
        }
    });
}
} // namespace crash::modelio::beam18::detail
