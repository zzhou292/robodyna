#pragma once
#include "case/vehicle_startup/physical_model/VehiclePhysicalModel.h"
#include "modelio/vehicle_sections/VehicleSectionResolution.h"
#include "lib_src/assembly/ShellPhysicalBinding.h"
#include "modelio/source_assembly/Law1ExecutionPolicy.h"

namespace crash::cases::vehicle_startup::shell_execution {
using modelio::assembly::Law1ExecutionProfile;
using modelio::assembly::Law1ExecutionPolicy;
struct Limits {
    // Inclusive source/startup reservation, deliberately overcharging shared
    // backing across native module caps. This is not expected resident RSS.
    std::size_t host_bytes = std::size_t{12} << 30;
    tl::fea::ShellPlasticityCatalogLimits catalog = tl::fea::ShellPlasticityCatalogLimits::Vehicle();
    tl::fea::ShellBatchFailureLimits failure = tl::fea::ShellBatchFailureLimits::Vehicle();
    tl::fea::ShellExecutionLimits execution = tl::fea::ShellExecutionLimits::Vehicle();
    tl::fea::ShellPhysicalBindingLimits physical = tl::fea::ShellPhysicalBindingLimits::Vehicle();
};
struct Forecast {
    std::size_t source_bytes = 0, fixed_bytes = 0, packing_bytes = 0;
    std::size_t native_reservation = 0, total_bytes = 0;
};

// Complete source-ordered immutable material/failure/role composition on the
// physical model's exact ledger. No clock, state, DOFs or force publication.
class VehicleShellExecution {
  public:
    static Forecast Preflight(const physical_model::VehiclePhysicalModel&, Limits = {});
    static VehicleShellExecution Prepare(const physical_model::VehiclePhysicalModel&, Limits = {});
    // Third argument stays explicit: historical Prepare(model,{}) is unambiguous.
    static Forecast Preflight(const physical_model::VehiclePhysicalModel&,Law1ExecutionProfile,Limits);
    static VehicleShellExecution Prepare(const physical_model::VehiclePhysicalModel&,Law1ExecutionProfile,Limits);
    const Law1ExecutionPolicy& law1_policy() const noexcept;
    const physical_model::VehiclePhysicalModel& model() const noexcept;
    const VehicleSectionResolution& resolution() const noexcept;
    const tl::fea::ShellBatchPlasticityBinding& catalog() const noexcept;
    const tl::fea::ShellBatchFailureBinding& failure() const noexcept;
    const tl::fea::ShellExecutionBinding& execution() const noexcept;
    const tl::fea::ShellPhysicalBinding& physical() const noexcept;
    const Forecast& forecast() const noexcept;
  private:
    struct Storage;
    explicit VehicleShellExecution(std::shared_ptr<const Storage> value) : storage_(std::move(value)) {}
    std::shared_ptr<const Storage> storage_;
};
} // namespace crash::cases::vehicle_startup::shell_execution
