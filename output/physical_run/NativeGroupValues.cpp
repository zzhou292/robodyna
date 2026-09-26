#include "NativeGroupValues.h"
#include "output/ArtifactIO.h"
#include <algorithm>
namespace crash::output::physical_run {
namespace {
NativeContactLayout Layout(NativeContactRole role) {
    return role==NativeContactRole::Self?NativeContactLayout::ExpandedSurface:NativeContactLayout::OrdinaryTwoSided;
}
bool Empty(const NativeGroupEntry& e) {
    const auto& v=e.publication;
    return e.role==NativeContactRole::Unspecified && !v.source_id && !v.topology_generation && !v.source_generation &&
        !v.nodes && !v.secondaries && !v.primary_mains && !v.expanded_mains && !v.publication_generation &&
        !v.reference_generation && !v.force_base_epoch && Bits(v.force_base_time)==Bits(0.) && Bits(v.force_base_velocity_time)==Bits(0.);
}
void Fields(std::vector<std::string>& out,std::size_t slot,const std::vector<std::string>& fields) {
    for(const auto& name:fields)out.push_back("native_group_"+std::to_string(slot)+"_"+name.substr(std::string("native_contact_").size()));
}
}
void CheckNativeGroupValues(const NativeGroupValues& group) {
    Require(group.count && group.count<=NativeGroupCapacity,"Native group count is invalid");
    for(std::size_t i=0;i<NativeGroupCapacity;++i) {
        const auto& e=group.entries[i];
        if(i>=group.count){Require(Empty(e),"Native group has nonempty trailing storage");continue;}
        Require(e.role==NativeContactRole::Self || e.role==NativeContactRole::MeshWall,"Native group role is invalid");
        CheckNativeContactValues(e.publication,Layout(e.role));
        for(std::size_t j=0;j<i;++j) {
            const auto& prior=group.entries[j];
            Require(e.role!=prior.role && e.publication.source_id!=prior.publication.source_id &&
                e.publication.nodes==prior.publication.nodes,"Native interfaces must have distinct roles/IDs in one physical domain");
        }
    }
}
void CheckNativeGroupContinuation(const NativeGroupValues& before,const NativeGroupValues& after) {
    CheckNativeGroupValues(before);CheckNativeGroupValues(after);
    Require(before.count==after.count,"Native group count changes across accepted intervals");
    for(std::size_t i=0;i<after.count;++i) {
        const auto& a=before.entries[i];const auto& b=after.entries[i];const auto& x=a.publication;const auto& y=b.publication;
        Require(a.role==b.role && SameNativeSource(x,y) && x.publication_generation<UINT64_MAX &&
            y.publication_generation==x.publication_generation+1 && y.reference_generation>=x.reference_generation &&
            y.reference_generation-x.reference_generation<=1,"Native group source/order or generation changed");
    }
}
std::vector<std::string> NativeGroupIntegerFields() {
    std::vector<std::string> out{"native_group_count"};
    for(std::size_t i=0;i<NativeGroupCapacity;++i) {
        out.push_back("native_group_"+std::to_string(i)+"_role");Fields(out,i,NativeContactIntegerFields());
    }
    return out;
}
std::vector<std::string> NativeGroupRealFields() {
    std::vector<std::string> out;for(std::size_t i=0;i<NativeGroupCapacity;++i)Fields(out,i,NativeContactRealFields());return out;
}
void EncodeNativeGroup(const NativeGroupValues& group,std::uint64_t* integers,double* reals) {
    CheckNativeGroupValues(group);std::fill_n(integers,NativeGroupIntegerCount,0);std::fill_n(reals,NativeGroupRealCount,0.);
    integers[0]=group.count;
    for(std::size_t i=0;i<group.count;++i) {
        const auto& e=group.entries[i];auto* row=integers+1+i*(1+NativeContactIntegerCount);
        row[0]=static_cast<std::uint64_t>(e.role);EncodeNativeContact(e.publication,row+1,reals+i*NativeContactRealCount,Layout(e.role));
    }
}
NativeGroupValues DecodeNativeGroup(const std::uint64_t* integers,const double* reals) {
    NativeGroupValues out;out.count=integers[0];Require(out.count && out.count<=NativeGroupCapacity,"Invalid encoded native group count");
    for(std::size_t i=0;i<NativeGroupCapacity;++i) {
        const auto* row=integers+1+i*(1+NativeContactIntegerCount);const auto* values=reals+i*NativeContactRealCount;
        if(i<out.count) {
            auto& e=out.entries[i];e.role=static_cast<NativeContactRole>(row[0]);
            e.publication=DecodeNativeContact(row+1,values,Layout(e.role));
        } else {
            for(unsigned j=0;j<1+NativeContactIntegerCount;++j)Require(!row[j],"Nonzero unused native group integer");
            for(unsigned j=0;j<NativeContactRealCount;++j)Require(Bits(values[j])==Bits(0.),"Nonzero unused native group scalar");
        }
    }
    CheckNativeGroupValues(out);return out;
}
}
