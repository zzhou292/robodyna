#include "Internal.h"
#include "lib_utils/BoundedArena.h"
namespace crash::modelio::solid_control_packets::detail {
Forecast Preflight(std::size_t bytes,Limits limits) {
    const Limits hard;
    Require(bytes&&bytes<=limits.file_bytes&&limits.file_bytes<=hard.file_bytes&&
        limits.parents&&limits.parents<=hard.parents&&limits.packets&&limits.packets<=hard.packets&&
        limits.partitions&&limits.partitions<=hard.partitions&&limits.retained_bytes&&
        limits.retained_bytes<=hard.retained_bytes&&limits.startup_bytes&&limits.startup_bytes<=hard.startup_bytes,
        "Invalid native solid packet resource limits");
    tl::util::BoundedArenaLayout peak(limits.startup_bytes);
    tl::util::ArenaRegion unused;
    Require(limits.retained_bytes>=sizeof(Values)+2048,"Native packet header exceeds retained cap");
    // Bound file/string/iterative DOM work before reading or parsing JSON.
    Require(peak.Append<unsigned char>(bytes,unused)&&peak.Append<unsigned char>(31*bytes,unused)&&
        peak.Append<unsigned char>(256u<<10,unused),"Native packet JSON workspace exceeds cap");
    const auto parse=peak.bytes();
    Require(peak.Append<unsigned char>(limits.retained_bytes,unused)&&
        peak.Append<unsigned char>(tl::util::SourceIdentityIndex<0>::Bytes(limits.parents),unused)&&
        peak.Append<std::uint8_t>(limits.parents,unused),"Native packet complete startup exceeds cap");
    return {parse,limits.retained_bytes,peak.bytes()};
}
std::size_t Owned(const Values& values) {
    return sizeof(Values)+values.parents.capacity()*sizeof(control::SourceParent)+
        values.parent_families.capacity()*sizeof(control::Family)+values.partitions.capacity()*sizeof(control::NativePartition)+
        values.packets.capacity()*sizeof(control::NativePacket)+values.ordered_element_ids.capacity()*sizeof(std::uint64_t);
}
control::Input View(const Values& values,std::uint64_t instance) {
    Require(instance!=0,"Native solid packets require the fresh Model source instance");
    control::Input out;out.profile=control::Profile::SourceDeclared;out.source_instance_id=instance;
    out.units=values.units;out.native_nvsiz=values.native_nvsiz;out.compiled_mvsiz=values.compiled_mvsiz;
    out.parents={values.parents.data(),values.parents.size()};
    out.partitions={values.partitions.data(),values.partitions.size()};
    out.packets={values.packets.data(),values.packets.size()};
    out.ordered_element_ids={values.ordered_element_ids.data(),values.ordered_element_ids.size()};return out;
}
} // namespace crash::modelio::solid_control_packets::detail
