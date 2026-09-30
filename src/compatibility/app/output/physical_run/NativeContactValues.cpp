#include "NativeContactValues.h"
#include "output/ArtifactIO.h"
#include <cmath>
namespace crash::output::physical_run {
namespace {
struct Field {const char* name;std::uint64_t NativeContactValues::*member;};
constexpr Field Fields[]{
    {"native_contact_source_id",&NativeContactValues::source_id},
    {"native_contact_topology_generation",&NativeContactValues::topology_generation},
    {"native_contact_source_generation",&NativeContactValues::source_generation},
    {"native_contact_nodes",&NativeContactValues::nodes},
    {"native_contact_secondaries",&NativeContactValues::secondaries},
    {"native_contact_primary_mains",&NativeContactValues::primary_mains},
    {"native_contact_expanded_mains",&NativeContactValues::expanded_mains},
    {"native_contact_publication_generation",&NativeContactValues::publication_generation},
    {"native_contact_reference_generation",&NativeContactValues::reference_generation},
    {"native_contact_force_base_epoch",&NativeContactValues::force_base_epoch}};
static_assert(std::size(Fields)==NativeContactIntegerCount);
}
void CheckNativeContactValues(const NativeContactValues& v,NativeContactLayout layout) {
    const bool expansion=layout==NativeContactLayout::OrdinaryTwoSided ? v.expanded_mains==2*v.primary_mains :
        layout==NativeContactLayout::ExpandedSurface && v.expanded_mains>=v.primary_mains && v.expanded_mains<=2*v.primary_mains;
    Require(v.source_id&&v.topology_generation&&v.source_generation&&v.nodes&&v.secondaries&&v.secondaries<=v.nodes&&
        v.primary_mains&&v.primary_mains<=UINT64_MAX/2&&expansion&&
        v.publication_generation&&v.reference_generation&&v.reference_generation<=v.publication_generation&&std::isfinite(v.force_base_time)&&v.force_base_time>=0&&
        std::isfinite(v.force_base_velocity_time)&&v.force_base_velocity_time>=0&&v.force_base_velocity_time<=v.force_base_time,
        "Native contact source or accepted force phase is invalid");
}
bool SameNativeSource(const NativeContactValues& a,const NativeContactValues& b) noexcept {
    return a.source_id==b.source_id&&a.topology_generation==b.topology_generation&&a.source_generation==b.source_generation&&
        a.nodes==b.nodes&&a.secondaries==b.secondaries&&a.primary_mains==b.primary_mains&&a.expanded_mains==b.expanded_mains;
}
std::vector<std::string> NativeContactIntegerFields(){std::vector<std::string> result;for(const auto& f:Fields)result.push_back(f.name);return result;}
std::vector<std::string> NativeContactRealFields(){return {"native_contact_force_base_time_s","native_contact_force_base_velocity_time_s"};}
void EncodeNativeContact(const NativeContactValues& v,std::uint64_t* integers,double* reals,NativeContactLayout layout) {
    CheckNativeContactValues(v,layout);for(std::size_t i=0;i<NativeContactIntegerCount;++i)integers[i]=v.*Fields[i].member;
    reals[0]=v.force_base_time;reals[1]=v.force_base_velocity_time;
}
NativeContactValues DecodeNativeContact(const std::uint64_t* integers,const double* reals,NativeContactLayout layout) {
    NativeContactValues v;for(std::size_t i=0;i<NativeContactIntegerCount;++i)v.*Fields[i].member=integers[i];
    v.force_base_time=reals[0];v.force_base_velocity_time=reals[1];CheckNativeContactValues(v,layout);return v;
}
}
