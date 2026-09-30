#include "Internal.h"
#include "modelio/vehicle_source/OriginalAuthority.h"
#include <algorithm>
#include <type_traits>
namespace crash::modelio::beam18::detail {
bool Selected(std::uint64_t id){return std::find(std::begin(Parts),std::end(Parts),id)!=std::end(Parts);}
void AddBytes(std::size_t& bytes,std::size_t count,std::size_t width,std::size_t cap) {
    Require(width && bytes<=cap && count<=(cap-bytes)/width,"Beam18 source byte cap exceeded");bytes+=count*width;
}
Forecast Budget(const source::CanonicalData& s,Policy policy,Limits limits) {
    const Limits hard;
    const std::size_t values[]{limits.host_bytes,limits.member_bytes,limits.metadata_bytes,limits.parents,
        limits.nodes,limits.canonical_nodes,limits.source_beams,limits.blocks};
    const std::size_t maximum[]{hard.host_bytes,hard.member_bytes,hard.metadata_bytes,hard.parents,
        hard.nodes,hard.canonical_nodes,hard.source_beams,hard.blocks};
    for(unsigned i=0;i<std::size(values);++i) Require(values[i] && values[i]<=maximum[i],"Invalid beam18 source limits");
    Require(policy==Policy::OriginalCircularFourPointLaw44V1,"Unsupported beam18 source policy");
    vehicle::CheckOriginalYarisAuthority(s);
    const auto& records=source::FindArray(s,"beams_records");
    Require(limits.parents>=142 && limits.nodes>=147 && s.canonical_nodes<=limits.canonical_nodes &&
        records.descriptor.layout.columns==10 && records.descriptor.layout.rows<=limits.source_beams &&
        s.inputs.source_member.bytes<=limits.member_bytes,"Beam18 source extent exceeds cap");
    Forecast f;auto add=[&](std::size_t n,std::size_t width){AddBytes(f.total_bytes,n,width,limits.host_bytes);};
    add(sizeof(Data)+sizeof(Source)+sizeof(source::CanonicalData)+512,1);
    for(const auto& a:s.arrays)add(a.bytes.capacity()+1,1);
    add(s.parts.capacity(),sizeof(source::PartDeclaration));
    add(s.selected_parts.capacity()+s.excluded_parts.capacity(),sizeof(std::uint64_t));
    add(s.canonical_bytes.size()+s.scope_bytes.size(),7);add(s.inputs.source_member.bytes,1);
    add(limits.metadata_bytes,4);add(limits.blocks,256);
    add(s.canonical_nodes,96);add(records.descriptor.layout.rows,160);
    add(limits.parents,sizeof(Row)+256);add(limits.nodes,sizeof(Node)+128);
    return f;
}
std::size_t OwnedPayload(const Data& d,Limits limits) {
    std::size_t bytes=sizeof(Data)+sizeof(Source)+256;
    auto vector=[&](const auto& v){AddBytes(bytes,v.capacity(),sizeof(typename std::decay_t<decltype(v)>::value_type),limits.host_bytes);};
    auto text=[&](const auto& s){AddBytes(bytes,s.capacity()+1,1,limits.host_bytes);};
    vector(d.parts);vector(d.rows);vector(d.nodes);vector(d.canonical_endpoints);
    vector(d.plastic_strain);vector(d.yield_stress_pa);vector(d.sources);
    for(const auto& r:d.rows)text(r.raw_card);
    for(const auto& n:d.nodes)text(n.raw_card);
    for(const auto& s:d.sources){text(s.block.filename);text(s.block.keyword);text(s.block.raw_text);text(s.block.sha256);
        vector(s.cards);for(const auto& c:s.cards)text(c.second);}
    return bytes;
}
} // namespace crash::modelio::beam18::detail
