#include "Internal.h"
#include <algorithm>
#include <map>
#include <set>
namespace crash::modelio::vehicle::rigid_part::detail {
namespace {
template<class T> std::vector<T> Decode(const source::CanonicalData& d,const char* name) {
    const auto& a=source::FindArray(d,name);
    return output::arrays::Decode<T>(a.descriptor,a.bytes);
}
}
void Geometry(SourceData& out,const source::CanonicalData& d,Limits limits) {
    const auto ids=Decode<std::uint64_t>(d,"node_ids");
    Require(ids.size()==d.canonical_nodes && std::is_sorted(ids.begin(),ids.end()),"Rigid source node map changed");
    std::map<std::uint64_t,std::size_t> parts;
    std::vector<std::set<std::uint64_t>> members(out.bodies.size());
    std::vector<std::size_t> counts(out.bodies.size(),0);
    for (std::size_t b=0;b<out.bodies.size();++b) parts.emplace(out.bodies[b].source_part_id,b);
    struct Family { const char* name; unsigned width,slots; bool shell; };
    for (const auto family:{Family{"shells_records",6,4,true},Family{"beams_records",10,2,false},
                            Family{"solids_records",10,8,false}}) {
        const auto records=Decode<std::uint64_t>(d,family.name);
        Require(records.size()%family.width==0,"Rigid source element extent changed");
        for (std::size_t e=0;e<records.size()/family.width;++e) {
            const auto* row=records.data()+family.width*e;
            const auto found=parts.find(row[1]);
            if (found==parts.end()) continue;
            Require(family.shell,"Original rigid PART has non-shell elements");
            const auto b=found->second;
            ++counts[b];
            for (unsigned n=0;n<family.slots;++n) {
                Require(std::binary_search(ids.begin(),ids.end(),row[2+n]),"Rigid PART references missing source node");
                members[b].insert(row[2+n]);
            }
        }
    }
    std::size_t total=0,extra=0,shells=0;
    for (std::size_t b=0;b<out.bodies.size();++b) {
        auto& body=out.bodies[b];
        const auto original=source::FindPart(d,body.source_part_id);
        Require(original.material==body.declaration.material.id && original.section==body.declaration.section.id,
                "Rigid canonical PART association changed");
        Require(!members[b].empty() && members[b].size()<=limits.members-total-extra,"Rigid PART member cap exceeded");
        body.part_nodes.assign(members[b].begin(),members[b].end());
        total+=body.part_nodes.size();shells+=counts[b];
        Require(counts[b]==body.shell_count,"Rigid per-PART shell coverage changed");
        Require(body.extra_nodes.size()<=limits.members-total-extra,"Rigid extra member cap exceeded");
        extra+=body.extra_nodes.size();
        for (auto node:body.extra_nodes)
            Require(std::binary_search(ids.begin(),ids.end(),node),"Rigid extra node absent from canonical source");
    }
    Require(total==5150 && extra==302 && shells==out.shell_count,"Original rigid PART incidence changed");
}
void InitializeTopology(const SourceData& d,tl::fea::rigid::NodalRigidPartTopology& output,Limits limits) {
    namespace r=tl::fea::rigid;
    std::vector<r::PartTopologyPartInput> parts;
    std::vector<r::PartTopologyExtraInput> extras;
    std::vector<std::uint64_t> expected;
    for (const auto& b:d.bodies) {
        parts.push_back({b.source_part_id,b.part_nodes.data(),b.part_nodes.size()});
        expected.insert(expected.end(),b.part_nodes.begin(),b.part_nodes.end());
        if (!b.extra_nodes.empty()) {
            extras.push_back({b.source_part_id,b.node_set_id,b.extra_nodes.data(),b.extra_nodes.size()});
            expected.insert(expected.end(),b.extra_nodes.begin(),b.extra_nodes.end());
        }
    }
    std::sort(expected.begin(),expected.end()); // Coverage order, never a mass reduction.
    r::PartTopologyInput in;
    in.source_instance_id=0x67208317e6c8eb1dULL;
    in.parts=parts.data();in.part_count=parts.size();in.extras=extras.data();in.extra_count=extras.size();
    in.merges=d.merges.data();in.merge_count=d.merges.size();
    in.expected_members=expected.data();in.expected_member_count=expected.size();
    in.other_rigid_members=d.plain_rigid_members.data();in.other_rigid_member_count=d.plain_rigid_members.size();
    in.limits.max_parts=limits.parts;in.limits.max_members=limits.members;
    in.limits.max_other_rigid_members=limits.members;
    const auto report=output.Initialize(in);
    Require(bool(report),report.message);
}
}
