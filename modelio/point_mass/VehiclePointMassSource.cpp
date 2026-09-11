#include "Mapping.h"
#include "modelio/physical_scope/CanonicalDomain.h"
#include "output/ArtifactIO.h"

namespace crash::modelio::point_mass {
struct VehiclePointMassSource::Storage {
    explicit Storage(const physical_scope::PhysicalScope& value) : source(value) {}
    physical_scope::PhysicalScope source;
    tl::fea::ElementMassContributions contributions;
    std::vector<Disposition> dispositions;
    Forecast forecast;
};
VehiclePointMassSource VehiclePointMassSource::Prepare(const physical_scope::PhysicalScope& source,
    const tl::fea::NodalNodeDomain& domain, Limits limits) {
    const auto forecast = Preflight(source, domain, limits);
    physical_scope::ValidateDeclaredDomain(source, domain);
    auto mapped = detail::Map(source.point_mass_source().data().records, domain, limits.native.max_records);
    // The canonical baseline includes every earlier PART point mass. Preserve
    // that selection while admitting all other real cards on declared nodes.
    for (const auto& original : source.point_mass_source().data().consumed)
        output::Require(original.record < mapped.dispositions.size() &&
            mapped.dispositions[original.record].domain_node != SIZE_MAX,
            "Declared point-mass domain omits an original retained PART mass");
    auto next = std::make_shared<Storage>(source);
    const auto scale = source.tied_source().canonical().data().inputs.units.mass_to_kg;
    const auto report = next->contributions.Initialize(domain,
        {domain.source_instance_id(), scale, mapped.retained.data(), mapped.retained.size()}, limits.native);
    output::Require(bool(report), report.message);
    output::Require(next->contributions.startup_payload_bytes() <= forecast.native_reservation,
                    "Point-mass native producer exceeded reservation");
    next->dispositions = std::move(mapped.dispositions);
    next->forecast = forecast;
    return VehiclePointMassSource(std::move(next));
}
const physical_scope::PhysicalScope& VehiclePointMassSource::source() const noexcept { return storage_->source; }
const tl::fea::ElementMassContributions& VehiclePointMassSource::contributions() const noexcept { return storage_->contributions; }
const std::vector<Disposition>& VehiclePointMassSource::dispositions() const noexcept { return storage_->dispositions; }
const Forecast& VehiclePointMassSource::forecast() const noexcept { return storage_->forecast; }
} // namespace crash::modelio::point_mass
