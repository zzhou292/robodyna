#include "Internal.h"
#include "output/BoundedArrayIO.h"

namespace crash::modelio::type45 {
struct VehicleType45Source::Storage {
    Storage(const physical_domain::VehiclePhysicalDomain& source, Policy p) : source(source), policy(p) {}
    physical_domain::VehiclePhysicalDomain source;
    Policy policy;
    Data data;
    Forecast forecast;
};
VehicleType45Source VehicleType45Source::Prepare(const physical_domain::VehiclePhysicalDomain& input, Policy policy, Limits limits) {
    const auto forecast = Preflight(input, policy, limits);
    const auto& canonical = input.source().tied_source().canonical().data();
    const auto& id_array = physical_scope::source::FindArray(canonical, "node_ids");
    const auto& xyz_array = physical_scope::source::FindArray(canonical, "node_positions");
    const auto ids = output::arrays::Decode<std::uint64_t>(id_array.descriptor, id_array.bytes);
    const auto xyz = output::arrays::Decode<double>(xyz_array.descriptor, xyz_array.bytes);
    auto next = std::make_shared<Storage>(input, policy);
    auto& data = next->data;
    data.rows = detail::Read(input.source().point_mass_source().rigid_source().data().sources, limits);
    const auto members = detail::Members(input);
    detail::Map(data.rows, ids, xyz, input.domain(), members);
    detail::CheckOriginal(data, policy);
    detail::ResolveProperties(data);
    data.owned_payload_bytes = sizeof(VehicleType45Source) + sizeof(Storage) + 64;
    detail::Add(data.owned_payload_bytes, data.rows.capacity(), sizeof(Row), limits.host_bytes);
    output::Require(data.owned_payload_bytes <= forecast.result_bytes, "TYPE45 source retained rows exceeded reservation");
    next->forecast = forecast;
    return VehicleType45Source(std::move(next));
}
const physical_domain::VehiclePhysicalDomain& VehicleType45Source::source_domain() const noexcept { return storage_->source; }
const Data& VehicleType45Source::data() const noexcept { return storage_->data; }
const Forecast& VehicleType45Source::forecast() const noexcept { return storage_->forecast; }
Policy VehicleType45Source::policy() const noexcept { return storage_->policy; }
} // namespace crash::modelio::type45
