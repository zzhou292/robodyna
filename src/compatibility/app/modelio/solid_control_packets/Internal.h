#pragma once
#include "NativePacketSource.h"
#include "output/BoundedArrayJson.h"
#include "lib_utils/SourceIdentityIndex.h"
namespace crash::modelio::solid_control_packets::detail {
using output::Require;
struct Values {
    control::UnitScale units;
    std::uint32_t native_nvsiz=0,compiled_mvsiz=0;
    std::vector<control::SourceParent> parents;
    std::vector<control::Family> parent_families;
    std::vector<control::NativePartition> partitions;
    std::vector<control::NativePacket> packets;
    std::vector<std::uint64_t> ordered_element_ids;
    std::size_t controlled_count=0;
};
Forecast Preflight(std::size_t bytes,Limits);
Values Read(const std::string&,const Artifact&,Limits);
void Bind(const Values&,const solid_source::Data&,const solid_control::EffectiveData&);
std::size_t Owned(const Values&);
control::Input View(const Values&,std::uint64_t);
control::Family Family(solid_source::Family);
} // namespace crash::modelio::solid_control_packets::detail
