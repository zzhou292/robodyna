#pragma once
#include "SourceAdmission.h"
#include "InterfaceFields.h"
#include "TiedRemovalSource.h"
#include "InitialModel.h"
#include "case/vehicle_dynamics/VehiclePhysicalDynamics.h"
#include "case/vehicle_self_contact/native/InitializerControlsSource.h"
#include <array>
#include <memory>
namespace crash::cases::vehicle_native_contact {
namespace n = tlfea::contact::radioss_type25;
namespace activity { class Declaration; }
using Role = vehicle_dynamics::native_contact::Role;
using OwnerSource = vehicle_wall::native::EnvelopeOwnerSource;
using SelfSource = vehicle_self_contact::native::mixed_starter::MixedStarterSource;
using WallSource = vehicle_wall::native::wall_interface::FiniteWallContactSource;
using ControlsSource = vehicle_self_contact::native::initial_controls::InitializerControlsSource;
struct Config {
    Config();
    n::ContactActivityPolicy activity = n::ContactActivityPolicy::AllActivePrefix;
    vehicle_dynamics::Config dynamics;
    double requested_duration_s = .002;
    // Independent plan ceilings, not permission for a larger live case. The
    // aggregate compares steady bytes to dynamics.startup.limits.device_bytes
    // and runtime-plus-one-initializer bytes to this separate peak limit.
    std::size_t peak_device_bytes = std::size_t{6} << 30;
    std::array<n::TransactionLimits, 2> transaction; // Self, mesh wall resource slots.
    std::array<n::initial_source::Limits, 2> initialization;
    detail::FieldPackingLimits fields;
    TiedRemovalLimits tied;
    std::size_t model_staging_bytes = 128u << 20;
};
struct Forecast {
    detail::SourceAdmission sources;
    vehicle_dynamics::Forecast physical;
    std::array<detail::InterfaceFieldForecast, 2> fields;
    TiedRemovalForecast tied;
    detail::InitialModelForecast model;
    std::array<n::initial_source::Forecast, 2> initialization;
    std::array<n::GeneralTransactionForecast, 2> contact;
    std::size_t case_metadata = 0, packing_retained = 0;
    std::size_t activity_metadata_reservation = 0, activity_workspace_reservation = 0;
    std::size_t host_preparation_ceiling = 0, prepared_source_retained = 0;
    std::size_t retained_host_bytes = 0, peak_host_bytes = 0;
    std::size_t steady_device_bytes = 0, peak_device_bytes = 0;
    // Conservative census host bound includes earlier source-preparation
    // scratch; device bound is only one sequential producer, without an owner.
    std::size_t census_peak_host_bytes = 0, census_peak_device_bytes = 0;
    bool fits_runtime_limits = false;
};
struct InitialCensus {
    Role role = Role::Self;
    n::initial_source::SeedIdentity identity;
    n::initial_source::Report result;
};
class PreparationError : public std::runtime_error {
  public:
    PreparationError(const char* stage, const n::initial_source::Report&);
    PreparationError(const char* stage, const n::TransactionReport&);
    n::initial_source::Report initial;
    n::TransactionReport transaction;
};
// Immutable host-prepared case. No live owner, accepted clock or device history
// is created by Prepare. Initialize reauthenticates and creates one fresh owner,
// then one declared-order native group. Calls/initializations are serialized.
class VehicleContactStartup {
  public:
    // Count/source preflight before field/PreparedSource allocation. Only the
    // preparation fields and host_preparation_ceiling are populated; contact,
    // initialization, aggregate runtime and census fields await Prepare.
    static Forecast ForecastPreparation(const OwnerSource&, const SelfSource&, const WallSource&,
                                        const ControlsSource&, Config = {});
    static VehicleContactStartup Prepare(const OwnerSource&, const SelfSource&, const WallSource&,
                                         const ControlsSource&, Config = {});
    const Forecast& forecast() const noexcept;
    const Config& config() const noexcept;
    const OwnerSource& owner_source() const noexcept;
    const SelfSource& self_source() const noexcept;
    const WallSource& wall_source() const noexcept;
    const ControlsSource& controls_source() const noexcept;
    const activity::Declaration* activity_source() const noexcept;
    tl::util::ConstView<detail::OrderedInterface> interface_order() const noexcept;
    // Immutable borrowed original operands, before BUC/history production.
    // Available for independent source qualification; lifetime is this case.
    n::initial_source::Input source_input(Role) const;
    // Read-only opaque preparation for independent qualification. No mutable
    // operands, history injection or physical acceptance are exposed.
    const n::initial_source::PreparedSource& prepared_source(Role) const;
    // Optional source-only census. Seeds are computed sequentially and retired;
    // none is supplied to runtime or published as a physical state.
    std::array<InitialCensus, 2> CensusInitialStates() const;
    vehicle_dynamics::VehiclePhysicalDynamics Initialize() const;
  private:
    struct Data;
    explicit VehicleContactStartup(std::shared_ptr<const Data> data) : data_(std::move(data)) {}
    std::shared_ptr<const Data> data_;
};
} // namespace crash::cases::vehicle_native_contact
