#pragma once
#include "case/vehicle_wall/native/FiniteWallContactSource.h"
#include "case/vehicle_self_contact/native/MixedStarterSource.h"
#include "case/vehicle_dynamics/native_contact/Observation.h"
namespace crash::cases::vehicle_self_contact::native::initial_controls { class InitializerControlsSource; }
namespace crash::cases::vehicle_native_contact::detail {
using OwnerSource = vehicle_wall::native::EnvelopeOwnerSource;
using SelfSource = vehicle_self_contact::native::mixed_starter::MixedStarterSource;
using WallSource = vehicle_wall::native::wall_interface::FiniteWallContactSource;
using ControlsSource = vehicle_self_contact::native::initial_controls::InitializerControlsSource;
struct SourceInputs {
    const OwnerSource& owner;
    const SelfSource& self;
    const WallSource& wall;
    const ControlsSource& controls;
};
struct OrderedInterface {
    vehicle_dynamics::native_contact::Role role;
    std::uint64_t native_id = 0;
    std::uint32_t native_storage_ordinal = 0;
};
struct SourceAdmission {
    std::array<OrderedInterface, 2> interfaces;
    std::size_t nodes = 0;
    std::size_t owner_retained = 0, self_retained = 0;
    std::size_t wall_incremental = 0, controls_incremental = 0;
    std::size_t retained_bytes = 0, construction_peak = 0;
};
// Host source identity/count/budget admission only. No initial rows, final
// removal CSR, transaction plan or live owner authority is published here.
SourceAdmission AdmitSources(const SourceInputs&, std::size_t host_cap);
} // namespace crash::cases::vehicle_native_contact::detail
