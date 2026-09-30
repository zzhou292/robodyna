#include "Storage.h"
#include "case/vehicle_dynamics/Storage.h"
#include "case/vehicle_dynamics/native_contact/Group.h"
#include "output/ArtifactIO.h"
namespace crash::cases::vehicle_native_contact {
vehicle_dynamics::VehiclePhysicalDynamics VehicleContactStartup::Initialize() const {
    const auto& data = *data_;
    output::Require(data.forecast.fits_runtime_limits,
                    "Complete native vehicle forecast exceeds the requested runtime limits; no owner allocated");
    // Reauthenticate shared immutable source handles before the one real owner
    // is constructed. GeneralInitialize repeats exact field/plan authentication.
    (void)detail::AdmitSources(data.sources(), data.config.dynamics.startup.limits.host_bytes);
    auto dynamics = vehicle_dynamics::VehiclePhysicalDynamics::Prepare(data.physical_source, data.config.dynamics);
    auto& state = dynamics.storage_->state();
    const tl::fea::ShellPhysicalParticipants participants{&state.qeph, &state.t3, &state.qbat,
        &state.type25, &state.type13, &state.solids, state.type45.get(), state.beam18.get()};
    const tl::fea::ShellPhysicalPublicationIdentity identity{state.config.configuration_id,
        state.config.qualification_id, state.source.startup()};
    std::array<vehicle_dynamics::native_contact::GroupInput, tl::fea::MaxNativeContactInterfaces> group;
    std::size_t count = 0;
    for (const auto& entry : data.forecast.sources.interfaces) {
        const auto i = detail::RoleIndex(entry.role);
        auto transaction = std::make_unique<n::Transaction>();
        const auto law = data.ContactConfig(entry.role);
        n::TransactionReport report;
        if (entry.role == Role::Self)
            report = transaction->GeneralInitialize(law, data.Self(), data.prepared[i], state.owner,
                state.publication, state.source.physical(), participants, identity, data.config.transaction[i]);
        else
            report = transaction->GeneralInitialize(law, data.Wall(), data.wall.fixed_ready(), data.prepared[i],
                state.owner, state.publication, state.source.physical(), participants, identity, data.config.transaction[i]);
        if (report.status != n::TransactionStatus::Ok)
            throw PreparationError(entry.role == Role::Self ? "Self runtime initialization" : "Wall runtime initialization", report);
        output::Require(transaction->allocations().device_bytes == data.forecast.contact[i].transaction.device_bytes,
                        "Actual native transaction bytes differ from its shared complete preflight");
        group[count++] = {entry.role, std::move(transaction)};
    }
    dynamics.InstallNativeContact(vehicle_dynamics::native_contact::Group::Adopt(std::move(group), count));
    const auto* installed = dynamics.native_contact_group();
    output::Require(installed && installed->count() == count, "Native activity group was not installed");
    const std::uint64_t initial_activity = data.config.activity == n::ContactActivityPolicy::ShellRemoval ? 1 : 0;
    for (std::size_t i = 0; i < count; ++i) {
        const auto selected = installed->transaction(i).accepted();
        output::Require(selected.available && selected.selectors.activity == 0 &&
                            selected.selectors.activity_generation == initial_activity &&
                            selected.selectors.reference_activity_generation == 0,
                        "Installed native source activity differs from the declared case policy");
    }
    output::Require(dynamics.allocations().device_bytes == data.forecast.steady_device_bytes &&
                        !dynamics.allocations().device_allocation_count_complete,
                    "Complete physical/native device byte accounting differs from the actual installed group");
    dynamics.storage_->forecast.peak_host_upper_bound = data.forecast.peak_host_bytes;
    return dynamics;
}
} // namespace crash::cases::vehicle_native_contact
