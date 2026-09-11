#include "Storage.h"
#include "lib_utils/BoundedArena.h"
#include "output/ArtifactIO.h"
namespace crash::cases::vehicle_startup::joints {
std::size_t VehicleJointModel::additional_owned_payload_bytes() const {
    const auto& s=*storage_;
    const auto& rigid=*s.model.rigid_binding();
    const auto shared=rigid.owned_payload_bytes();
    const auto native=s.model.owned_payload_bytes();
    const auto source=s.source.data().owned_payload_bytes;
    output::Require(shared>=sizeof(rigid) && native>=shared-sizeof(rigid)+sizeof(s.model) &&
        source>=sizeof(s.source),"Joint retained source partitions are inconsistent");
    tl::util::BoundedArenaLayout bytes(Limits{}.host_bytes);
    tl::util::ArenaRegion unused;
    output::Require(bytes.Append<std::byte>(sizeof(VehicleJointModel)+sizeof(Storage)+64,unused) &&
        bytes.Append<std::byte>(source-sizeof(s.source),unused) &&
        bytes.Append<std::byte>(native-(shared-sizeof(rigid))-sizeof(s.model),unused) &&
        bytes.Append<std::uint32_t>(s.rows.capacity(),unused),
        "Additional retained joint source exceeds its complete cap");
    return bytes.bytes();
}
} // namespace crash::cases::vehicle_startup::joints
