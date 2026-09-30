#include "Storage.h"
#include "modelio/type45/SourcePolicy.h"
#include "lib_utils/BoundedArena.h"
namespace crash::cases::vehicle_startup::joints {
Forecast VehicleJointModel::Preflight(const Physical& physical,const Source& source,Limits limits) {
    const auto& domain=physical.source_domain().domain();
    const auto& rigid=physical.rigid_assembly();
    const auto& data=source.data();
    output::Require(limits.host_bytes && limits.host_bytes<=Limits{}.host_bytes &&
        domain.SharesStorage(source.source_domain().domain()) && rigid.domain()->SharesStorage(domain) &&
        source.source_domain().policy()==modelio::type45::detail::DomainPolicy(source.policy()) &&
        physical.source_domain().policy()==source.source_domain().policy() && data.rows.size()==44 &&
        data.required==modelio::type45::detail::Required(source.policy()) && data.boundaries==44-data.required,
        "Joint source must retain the exact complete physical domain and original census");
    const tl::fea::type45::ModelLimits hard;
    output::Require(limits.model.max_joints>=data.required && limits.model.max_joints<=hard.max_joints &&
        limits.model.max_nodes>=domain.node_count() && limits.model.max_nodes<=hard.max_nodes &&
        limits.model.max_host_bytes && limits.model.max_host_bytes<=hard.max_host_bytes,
        "Invalid retained joint model capacities");
    Forecast result;
    result.physical_reservation=physical.forecast().total_bytes;
    result.source_reservation=source.forecast().total_bytes;
    result.model_reservation=limits.model.max_host_bytes;
    tl::util::BoundedArenaLayout packing(limits.host_bytes),total(limits.host_bytes);
    tl::util::ArenaRegion region;
    output::Require(packing.Append<std::byte>(sizeof(Storage)+sizeof(VehicleJointModel)+256,region) &&
        packing.Append<tl::fea::type45::JointInput>(data.required,region) &&
        packing.Append<std::uint32_t>(data.required,region),"Joint packing exceeds capacity");
    result.packing_bytes=packing.bytes();
    // Deliberately charge upstream phase reservations and the native model's
    // whole ceiling separately, even where immutable handles share backing.
    for(auto bytes:{result.physical_reservation,result.source_reservation,
                   result.model_reservation,result.packing_bytes})
        output::Require(total.Append<std::byte>(bytes,region),"Complete joint startup exceeds host cap");
    result.total_bytes=total.bytes();
    return result;
}
} // namespace crash::cases::vehicle_startup::joints
