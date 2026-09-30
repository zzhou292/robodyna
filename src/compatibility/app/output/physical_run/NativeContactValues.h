#pragma once
#include <cstdint>
#include <string>
#include <vector>
namespace crash::output::physical_run {
// Read-only accepted publication provenance, not restart state or V5 policy.
// No last-attempt counters/forces are mislabeled as accepted diagnostics.
struct NativeContactValues {
    std::uint64_t source_id=0,topology_generation=0,source_generation=0;
    std::uint64_t nodes=0,secondaries=0,primary_mains=0,expanded_mains=0;
    std::uint64_t publication_generation=0,reference_generation=0,force_base_epoch=0;
    double force_base_time=0,force_base_velocity_time=0;
};
inline constexpr std::size_t NativeContactIntegerCount=10,NativeContactRealCount=2;
// Ordinary records retain exact 2P admission. The explicit group profile also
// covers true P+S source cardinality; it does not claim a different contact law.
enum class NativeContactLayout { OrdinaryTwoSided, ExpandedSurface };
void CheckNativeContactValues(const NativeContactValues&,NativeContactLayout=NativeContactLayout::OrdinaryTwoSided);
bool SameNativeSource(const NativeContactValues&,const NativeContactValues&) noexcept;
std::vector<std::string> NativeContactIntegerFields();
std::vector<std::string> NativeContactRealFields();
void EncodeNativeContact(const NativeContactValues&,std::uint64_t*,double*,NativeContactLayout=NativeContactLayout::OrdinaryTwoSided);
NativeContactValues DecodeNativeContact(const std::uint64_t*,const double*,NativeContactLayout=NativeContactLayout::OrdinaryTwoSided);
}
