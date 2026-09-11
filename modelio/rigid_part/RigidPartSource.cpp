#include "Internal.h"
#include <algorithm>
namespace crash::modelio::vehicle::rigid_part {
struct RigidPartSource::Storage {
    explicit Storage(const VehicleSourcePlan& value):source(value) {}
    VehicleSourcePlan source;
    SourceData data;
    tl::fea::rigid::NodalRigidPartTopology topology;
};
std::size_t RigidPartSource::AdditionalForecast(const VehicleSourcePlan& source,Limits limits) {
    const Limits hard;
    detail::Require(limits.host_bytes && limits.host_bytes<=hard.host_bytes &&
        limits.member_bytes && limits.member_bytes<=hard.member_bytes &&
        limits.metadata_bytes && limits.metadata_bytes<=hard.metadata_bytes &&
        limits.members && limits.members<=hard.members && limits.parts && limits.parts<=hard.parts &&
        limits.blocks && limits.blocks<=hard.blocks,"Invalid rigid source capacity");
    detail::CheckOriginal(source);
    const auto& d=source.canonical().data();
    detail::Require(d.inputs.source_member.bytes<=limits.member_bytes && source.parts().size()<=limits.parts &&
                    limits.members>=7539,
                    "Rigid source input exceeds cap");
    std::size_t bytes=sizeof(Storage)+256;
    const auto add=[&](std::size_t count,std::size_t width) {
        detail::Require(bytes<=limits.host_bytes && count<=(limits.host_bytes-bytes)/width,
                        "Rigid source startup byte cap exceeded");
        bytes+=count*width;
    };
    add(d.inputs.source_member.bytes,1); // Borrowed original bytes coexist with startup.
    add(d.canonical_bytes.size(),7); // Metadata DOM, source requests and parse scratch.
    add(limits.metadata_bytes,8); // Raw cards, strings, vectors and decoding scratch.
    add(limits.blocks,256);
    add(limits.parts,sizeof(Body)+sizeof(std::size_t));
    add(limits.members,256); // PART/extra/other member vectors, sets and TL input staging.
    add(8*1024*1024,1); // Existing topology's complete owned+temporary cap.
    for (const auto* name:{"node_ids","shells_records","beams_records","solids_records"})
        add(source::FindArray(d,name).bytes.size(),2);
    return bytes;
}
RigidPartSource RigidPartSource::Prepare(const VehicleSourcePlan& source,const std::string& member,Limits limits) {
    const auto additional=AdditionalForecast(source,limits);
    detail::Require(source.startup_budget_bytes()<=limits.host_bytes-additional,"Rigid retained source exceeds cap");
    auto next=std::make_shared<Storage>(source);
    next->data=detail::Declarations(source,limits);
    detail::ReadSources(next->data,source.canonical().data(),member,limits);
    detail::Connections(next->data,limits);
    detail::Geometry(next->data,source.canonical().data(),limits);
    detail::InitializeTopology(next->data,next->topology,limits);
    detail::Require(next->topology.part_count()==22 && next->topology.root_count()==20 &&
                    next->topology.member_count()==5452,"Rigid source topology coverage changed");
    next->data.node_roots.reserve(next->topology.member_count());
    for (std::size_t r=0;r<next->topology.root_count();++r) {
        const auto& root=next->topology.roots()[r];
        for (std::size_t n=0;n<root.member_count;++n)
            next->data.node_roots.push_back({next->topology.root_members()[root.member_offset+n],r});
    }
    std::sort(next->data.node_roots.begin(),next->data.node_roots.end(),
              [](const auto& a,const auto& b) {return a.source_node_id<b.source_node_id;});
    next->data.startup_budget_bytes=source.startup_budget_bytes()+additional;
    return RigidPartSource(std::move(next));
}
const VehicleSourcePlan& RigidPartSource::source() const noexcept {return storage_->source;}
const SourceData& RigidPartSource::data() const noexcept {return storage_->data;}
const tl::fea::rigid::NodalRigidPartTopology& RigidPartSource::topology() const noexcept {return storage_->topology;}
const Body* RigidPartSource::body(std::size_t part) const noexcept {
    const auto& d=data();
    return part<d.part_to_body.size() && d.part_to_body[part]!=SIZE_MAX ? &d.bodies[d.part_to_body[part]] : nullptr;
}
std::size_t RigidPartSource::root_for_node(std::uint64_t node) const noexcept {
    const auto& nodes=data().node_roots;
    const auto found=std::lower_bound(nodes.begin(),nodes.end(),node,
        [](const auto& value,std::uint64_t id) {return value.source_node_id<id;});
    return found!=nodes.end() && found->source_node_id==node ? found->root_index : SIZE_MAX;
}
std::size_t RigidPartSource::root_index(std::size_t part) const noexcept {
    return body(part) ? topology().parts()[data().part_to_body[part]].root_index : SIZE_MAX;
}
}
