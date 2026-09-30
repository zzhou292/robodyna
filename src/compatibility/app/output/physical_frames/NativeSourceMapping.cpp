#include "NativeSourceMapping.h"
#include "output/full_shell/static_bundle/MappingArrays.h"
#include "lib_utils/BoundedArena.h"
namespace crash::output::physical_frames::detail {
namespace source=records::source;
std::size_t NativeMappingBytes(const source::PreparedSourceMapping& m,std::size_t cap) {
    tl::util::BoundedArenaLayout b(cap);tl::util::ArenaRegion unused;
    const auto n=m.nodes(),p=m.parents().size(),c=m.source().data().canonical_nodes;
    Require(n&&p&&n<=1048576&&p<=1048576&&c<=1048576&&
        b.Append<std::byte>(sizeof(NativeSourceMapping),unused)&&
        b.Append<std::uint64_t>(2*n,unused)&&b.Append<std::uint32_t>(3*n,unused)&&
        b.Append<double>(6*c,unused)&&b.Append<std::uint32_t>(14*p,unused)&&
        b.Append<ParentField>(p,unused),"Native archive mapping scratch exceeds cap");
    return b.bytes();
}
NativeSourceMapping BindNativeSource(const source::PreparedSourceMapping& m,
    const tl::fea::ShellPhysicalBinding& p,std::size_t cap) {
    NativeMappingBytes(m,cap);
    Require(p.prepared()&&p.execution()&&p.mapping()&&p.mapping()->Matches(*p.shells(),*p.domain())&&
        m.nodes()==p.domain()->node_count()&&p.shells()->qeph_count()&&p.shells()->t3_count()&&
        !p.shells()->qbat_count()&&m.parents().size()==p.shells()->qeph_count()+p.shells()->t3_count(),
        "Native archive mapping is not the complete physical QEPH/T3 source");
    const auto decode=[&](std::size_t i){const auto& a=m.arrays()[i];return arrays::Decode<std::uint32_t>(a.descriptor,a.bytes);};
    const auto& id_array=m.arrays()[source::detail::NodeIds];
    const auto ids=arrays::Decode<std::uint64_t>(id_array.descriptor,id_array.bytes);
    const auto canonical=decode(source::detail::NodeCanonical),refs=decode(source::detail::ParentReference),connectivity=decode(source::detail::ParentNodes);
    const auto& positions=source::FindArray(m.source().data(),"node_positions");
    const auto xyz=arrays::Decode<double>(positions.descriptor,positions.bytes);
    NativeSourceMapping result;result.nodes.reserve(ids.size());result.parents.reserve(m.parents().size());
    for(std::size_t i=0;i<ids.size();++i) {
        const auto index=p.domain()->Find(ids[i]);
        Require(index<p.domain()->node_count()&&index<=UINT32_MAX&&canonical[i]<xyz.size()/3,
            "Native render node is missing from physical source");
        const auto& x=p.domain()->nodes()[index].position;const auto c=3*std::size_t(canonical[i]);
        Require(Bits(x.x)==Bits(xyz[c])&&Bits(x.y)==Bits(xyz[c+1])&&Bits(x.z)==Bits(xyz[c+2]),
            "Native render reference position differs from physical source");
        result.nodes.push_back(std::uint32_t(index));
    }
    for(std::size_t i=0;i<m.parents().size();++i) {
        const auto family=refs[3*i+1],index=refs[3*i+2];
        Require(family==QephFamily||family==T3Family,"Native render source has unsupported participant");
        const auto native_family=family==QephFamily?tl::fea::ShellBindingFamily::Qeph:tl::fea::ShellBindingFamily::T3;
        const auto* role=p.execution()->parent(native_family,index);const auto& declared=m.parents()[i];
        Require(role&&role->source.source_parent_id==declared.source_element&&role->source.source_part_id==declared.source_part&&
            role->material_points==declared.native_points&&Plasticity(role->law)==declared.plastic,
            "Native render parent/material-point mapping differs from physical source");
        for(unsigned slot=0;slot<4;++slot) {
            const auto local=connectivity[4*i+slot];
            const auto shell=family==QephFamily?p.shells()->qeph_nodes(index)[slot]:p.shells()->t3_nodes(index)[slot<3?slot:2];
            Require(local<result.nodes.size()&&result.nodes[local]==p.mapping()->owner_index(shell),
                "Native render ordered connectivity differs from physical source");
        }
        result.parents.push_back({family,index,role->law});
    }
    return result;
}
}
