#pragma once
#include "EnvelopePhysicalSource.h"
#include "case/vehicle_startup/shell_execution/VehicleShellExecution.h"

namespace crash::cases::vehicle_wall::native {
using EnvelopeExecutionLimits = vehicle_startup::shell_execution::Limits;
struct EnvelopeExecutionForecast {
    std::size_t source_bytes=0,fixed_bytes=0,packing_bytes=0,native_reservation=0;
    std::size_t previous_source_peak=0,current_phase=0,total_bytes=0;
};
// Genuine material/failure/physical composition on the combined mechanical
// source. No fabricated original Vehicle wrapper, owner, clock or contact stage.
class EnvelopeExecutionSource {
  public:
    static EnvelopeExecutionForecast Preflight(const EnvelopePhysicalSource&, EnvelopeExecutionLimits = {});
    static EnvelopeExecutionSource Prepare(const EnvelopePhysicalSource&, EnvelopeExecutionLimits = {});
    const EnvelopePhysicalSource& mechanical() const noexcept;
    const tl::fea::ShellPhysicalBinding& physical() const noexcept;
    const tl::fea::ShellBatchPlasticityBinding& catalog() const noexcept;
    const tl::fea::ShellBatchFailureBinding& failure() const noexcept;
    const tl::fea::ShellExecutionBinding& execution() const noexcept;
    const modelio::assembly::Law1ExecutionPolicy& law1_policy() const noexcept;
    const EnvelopeExecutionForecast& forecast() const noexcept;
    std::size_t retained_host_upper_bound(std::size_t cap) const;
  private:
    struct Data;
    explicit EnvelopeExecutionSource(std::shared_ptr<const Data> value) : data_(std::move(value)) {}
    std::shared_ptr<const Data> data_;
};
} // namespace crash::cases::vehicle_wall::native
