#include "ReadInternal.h"
#include "output/BoundedArrayIO.h"
#include <algorithm>
#include <set>

namespace crash::modelio::type13::reader {
namespace {
native::Vec3 Vector(const Value& value) {
    Require(value.IsArray()&&value.Size()==3,"TYPE13 coordinate tuple shape changed");
    return {Real(value[0]),Real(value[1]),Real(value[2])};
}
double Scalar(const std::string& text,std::size_t offset,std::size_t width,unsigned bit,unsigned mask) {
    const auto field=text.substr(std::min(offset,text.size()),width);
    const bool blank=auxiliary::Trim(field).empty();
    Require(blank==bool(mask&(1u<<bit)),"TYPE13 raw source blank mask changed");
    return blank?0:auxiliary::Number(text,offset,width);
}
template<std::size_t N> void Append(std::string& text,const std::uint64_t (&values)[N]) {
    text+=output::arrays::Encode({output::arrays::Scalar::UInt64,1,N,{}},values,N,{2048,1,64});
}
void NodeAssociation(const Value& value,Node& node) {
    const auto& raw=node.raw_text;
    Require(raw.size()<=72&&auxiliary::Id(raw,0,8)==node.id,"TYPE13 raw source NODE identity changed");
    const double coordinates[3]={node.position_native.x,node.position_native.y,node.position_native.z};
    const double si[3]={node.position_m.x,node.position_m.y,node.position_m.z};
    for(unsigned i=0;i<3;++i) {
        Same(Scalar(raw,8+16*i,16,i+1,node.blank_mask),coordinates[i]);
        Same(coordinates[i]*.001,si[i]);
    }
    const auto& codes=Array(value,"codes",2,2);
    for(unsigned i=0;i<2;++i) {
        Same(Real(codes[i]),Scalar(raw,56+8*i,8,4+i,node.blank_mask));
        Require(Real(codes[i])==0,"TYPE13 source NODE motion codes are unsupported");
    }
    Require(!(node.blank_mask&1)&&node.blank_mask<=63,"TYPE13 node blank scope changed");
}
}
void ReadGeometry(const Value& document,ReadLimits limits,Data& data) {
    const auto& nodes=Array(document,"nodes",limits.nodes,7494);
    const auto& beams=Array(document,"beams",limits.beams,4442);
    Require(nodes.Size()==7494&&beams.Size()==4442,"TYPE13 original selected closure changed");
    data.nodes.reserve(nodes.Size());
    std::string node_identity;node_identity.reserve(7494*168);
    for(const auto& value:nodes.GetArray()) {
        Require(value.IsObject()&&value.MemberCount()==7,"TYPE13 source node declaration shape changed");
        Node node;node.id=Unsigned(value,"source_id",UINT32_MAX);node.source_line=Unsigned(value,"source_line",UINT32_MAX);
        node.blank_mask=Unsigned(value,"blank_mask",63);node.raw_text=Text(value,"raw_text");
        node.position_native=Vector(Member(value,"position_native"));node.position_m=Vector(Member(value,"position_m"));
        Require(node.id&&node.source_line&&
            (data.nodes.empty()||data.nodes.back().id<node.id),"TYPE13 source NODE ordering/line changed");
        NodeAssociation(value,node);
        const std::uint64_t row[]={node.id,node.source_line,node.blank_mask,
            output::Bits(node.position_native.x),output::Bits(node.position_native.y),output::Bits(node.position_native.z),
            output::Bits(node.position_m.x),output::Bits(node.position_m.y),output::Bits(node.position_m.z),0,0,node.raw_text.size()};
        Append(node_identity,row);node_identity+=node.raw_text;
        data.nodes.push_back(std::move(node));
    }
    Require(output::Sha256(node_identity)=="d1d51ae7bfcbcfa687f7ba14b21072e3de0b09abb3a7ca43bab390bae50f387a",
            "TYPE13 complete original coordinate/line inventory changed");
    std::vector<bool> used(data.nodes.size()),physical(data.nodes.size());
    std::set<std::uint64_t> ids;std::string endpoint_identity,beam_identity;
    endpoint_identity.reserve(4442*24);beam_identity.reserve(4442*200);
    data.beams.reserve(beams.Size());
    for(const auto& value:beams.GetArray()) {
        Require(value.IsObject()&&value.MemberCount()==7,"TYPE13 beam declaration shape changed");
        Beam beam;beam.id=Unsigned(value,"source_id",UINT32_MAX);beam.source_line=Unsigned(value,"source_line",UINT32_MAX);
        beam.canonical_index=Unsigned(value,"canonical_index",4684);beam.raw_text=Text(value,"raw_text");
        Require(beam.id&&ids.insert(beam.id).second&&beam.source_line&&beam.raw_text.size()<=80&&
            (data.beams.empty()||(data.beams.back().canonical_index<beam.canonical_index&&data.beams.back().source_line<beam.source_line)),
            "TYPE13 original beam ordering/identity changed");
        Require(Unsigned(value,"blank_mask")==480,"TYPE13 beam release declaration changed");
        const auto& row=Array(value,"raw_record",10,10);const auto& indices=Array(value,"node_indices",3,3);
        std::uint64_t fields[10];
        for(unsigned i=0;i<10;++i) {
            fields[i]=Unsigned(row[i],UINT32_MAX);
            Same(double(fields[i]),Scalar(beam.raw_text,8*i,8,i,480));
        }
        Require(fields[0]==beam.id&&fields[1]==2000486&&fields[2]!=fields[3]&&fields[4]==2000001&&fields[9]==2,
                "TYPE13 source beam endpoints/N3/LOCAL changed");
        native::ReferenceInput reference;
        for(unsigned i=0;i<3;++i) {
            beam.node_indices[i]=Unsigned(indices[i],data.nodes.size()-1);
            const auto& node=data.nodes[beam.node_indices[i]];
            Require(node.id==fields[2+i],"TYPE13 source beam node-index association changed");
            reference.position[i]=node.position_native;used[beam.node_indices[i]]=true;
            if(i<2)physical[beam.node_indices[i]]=true;
        }
        for(unsigned i=0;i<4;++i)Require(fields[5+i]==0,"TYPE13 source release is unsupported");
        Require(native::InitializeElement(data.property,reference,beam.startup)==native::Status::Success,
                "TYPE13 native source reference/coefficient initialization rejected");
        for(unsigned i=0;i<2;++i) {
            const auto& node=data.nodes[beam.node_indices[i]];const auto& actual=beam.startup.reference.position_m[i];
            Same(node.position_m.x,actual.x);Same(node.position_m.y,actual.y);Same(node.position_m.z,actual.z);
        }
        const std::uint64_t endpoints[]={beam.id,fields[2],fields[3]};Append(endpoint_identity,endpoints);
        const std::uint64_t original[]={beam.id,beam.source_line,beam.canonical_index,fields[0],fields[1],fields[2],fields[3],
            fields[4],fields[5],fields[6],fields[7],fields[8],fields[9],480,beam.raw_text.size()};
        Append(beam_identity,original);beam_identity+=beam.raw_text;
        data.beams.push_back(std::move(beam));
    }
    Require(std::all_of(used.begin(),used.end(),[](bool x){return x;})&&
        std::count(physical.begin(),physical.end(),true)==7493,"TYPE13 contains unused or missing original node roles");
    Require(output::Sha256(endpoint_identity)=="e9d2b1e5522392fd93696e4ee07d5c7dfcee1297aea6e0f8db43d5f23f590528",
            "TYPE13 complete original endpoint inventory changed");
    Require(output::Sha256(beam_identity)=="6768583917785f5b7a26602a963341a817405b61fa9ad7dd8ba18676607a2635",
            "TYPE13 complete original beam/line inventory changed");
}
}
