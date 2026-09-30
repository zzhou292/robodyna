#include "Storage.h"
#include "case/vehicle_dynamics/native_contact/Group.h"
#include "output/ArtifactIO.h"
#include <algorithm>
namespace crash::cases::vehicle_native_contact {
namespace {
std::size_t Extra(std::size_t total, std::size_t shared) {
    output::Require(total >= shared, "Native case published retained partition differs");
    return total - shared;
}
}
void VehicleContactStartup::Data::ForecastPreparation() {
    using detail::AddBytes;
    const auto input = sources();
    const auto cap = config.dynamics.startup.limits.host_bytes;
    detail::CheckConfig(config, controls);
    auto& f = forecast;
    f.sources = detail::AdmitSources(input, cap);
    f.physical = vehicle_dynamics::VehiclePhysicalDynamics::Preflight(physical_source, config.dynamics);
    f.tied = TiedRemovalSource::Preflight(owner, controls, config.tied);
    f.model = detail::InitialModel::Preflight(input, config.model_staging_bytes);
    f.fields[0] = detail::InterfaceFields::Preflight(input, Role::Self, config.fields);
    f.fields[1] = detail::InterfaceFields::Preflight(input, Role::MeshWall, config.fields);
    f.case_metadata = sizeof(Data) + sizeof(VehicleContactStartup) + 512 +
        sizeof(vehicle_dynamics::native_contact::Group) +
        2 * vehicle_dynamics::native_contact::Contribution::bookkeeping_payload_bytes();
    // The source facade is constructed from this exact retained owner handle.
    // Its owning API includes a small additional wrapper, charged explicitly.
    f.case_metadata = AddBytes(f.case_metadata,
        Extra(f.physical.startup.retained_source_upper_bound, f.sources.owner_retained));
    if (config.activity == n::ContactActivityPolicy::ShellRemoval) {
        const activity::Limits limits;
        f.activity_metadata_reservation = limits.metadata_bytes;
        f.activity_workspace_reservation = limits.workspace_bytes;
        f.case_metadata = AddBytes(f.case_metadata, limits.metadata_bytes);
    }
    f.packing_retained = AddBytes(AddBytes(f.tied.retained_bytes, f.model.retained_bytes), f.case_metadata);
    for (const auto& field : f.fields) f.packing_retained = AddBytes(f.packing_retained, field.retained_bytes);
    auto transient = f.tied.temporary_bytes;
    std::size_t preparation_reserve = 0;
    const n::initial_source::Limits hard;
    for (const auto& limit : config.initialization) {
        output::Require(limit.max_host_bytes && limit.max_host_bytes <= hard.max_host_bytes &&
                            limit.max_device_bytes && limit.max_device_bytes <= hard.max_device_bytes,
                        "Initializer preparation reservation exceeds its public ceiling");
        preparation_reserve = AddBytes(preparation_reserve, limit.max_host_bytes);
    }
    transient = std::max({transient, preparation_reserve, f.activity_workspace_reservation});
    f.host_preparation_ceiling = std::max(f.sources.construction_peak,
        AddBytes(AddBytes(f.sources.retained_bytes, f.packing_retained), transient));
    output::Require(f.host_preparation_ceiling <= cap,
                    "Complete host source/field/initializer preparation exceeds the case cap");
}
void VehicleContactStartup::Data::CompleteForecast() {
    using detail::AddBytes;
    auto& f = forecast;
    const auto source_extra = Extra(f.sources.retained_bytes, f.sources.owner_retained);
    for (std::size_t i = 0; i < prepared.size(); ++i) {
        f.initialization[i] = prepared[i].forecast();
        output::Require(f.initialization[i].status == n::initial_source::Status::Ok,
                        "Prepared initializer has no successful owning forecast");
        f.prepared_source_retained = AddBytes(f.prepared_source_retained, f.initialization[i].retained_host_bytes);
    }
    const auto census_retained = AddBytes(AddBytes(f.sources.retained_bytes, f.packing_retained), f.prepared_source_retained);
    f.census_peak_host_bytes = std::max(census_retained, f.host_preparation_ceiling);
    for (const auto& one : f.initialization) {
        f.census_peak_host_bytes = std::max(f.census_peak_host_bytes,
            AddBytes(census_retained, Extra(one.peak_host_bytes, one.retained_host_bytes)));
        f.census_peak_device_bytes = std::max(f.census_peak_device_bytes, one.peak_device_bytes);
    }
    const auto external = AddBytes(AddBytes(source_extra, f.packing_retained), f.prepared_source_retained);
    const auto physical_retained = AddBytes(f.physical.startup.retained_host_upper_bound, f.physical.workspace_bytes);
    auto host_live = AddBytes(physical_retained, external);
    auto device_live = AddBytes(f.physical.startup.device_bytes, f.physical.motion.device_bytes);
    f.peak_host_bytes = std::max(f.host_preparation_ceiling, AddBytes(f.physical.peak_host_upper_bound, external));
    f.peak_device_bytes = device_live;
    for (const auto& ordered : f.sources.interfaces) {
        const auto i = detail::RoleIndex(ordered.role);
        const auto& one = f.contact[i];
        // Both PreparedSources are already included in external. General's
        // host peak includes its own prepared source once; only that exact
        // published retained term is subtracted here.
        const auto startup_extra = Extra(one.peak_host_bytes, one.initializer.retained_host_bytes);
        f.peak_host_bytes = std::max(f.peak_host_bytes, AddBytes(host_live, startup_extra));
        f.peak_device_bytes = std::max(f.peak_device_bytes, AddBytes(device_live, one.peak_device_bytes));
        host_live = AddBytes(host_live, one.transaction.host_bytes);
        device_live = AddBytes(device_live, one.transaction.device_bytes);
    }
    f.retained_host_bytes = host_live;
    f.steady_device_bytes = device_live;
    f.peak_host_bytes = std::max(f.peak_host_bytes, host_live);
    f.fits_runtime_limits = f.peak_host_bytes <= config.dynamics.startup.limits.host_bytes &&
        f.steady_device_bytes <= config.dynamics.startup.limits.device_bytes &&
        f.peak_device_bytes <= config.peak_device_bytes;
}
} // namespace crash::cases::vehicle_native_contact
