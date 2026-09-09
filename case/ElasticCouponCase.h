#pragma once
#include "ElasticCouponAdmission.h"
#include "chrono/NodalMeshOutput.h"
#include <memory>

namespace crash::case_data {
struct ElasticCouponConfig {
    unsigned refinement = 1;  // Separate fixed-step runs: 1, 2 or 4.
    unsigned diagnostic_intervals = 20;
};
struct ElasticCouponStepRequest {
    // A caller may impose a stricter stop envelope on this attempt. It cannot
    // loosen the qualified case envelope; rejection preserves accepted state.
    double maximum_displacement = ElasticCouponLimits::displacement;
};
enum class CouponStatus { Ok, InvalidInput, NotInitialized, AlreadyInitialized,
                          StateFailure, ElementFailure, AdmissionFailure, AuditFailure, OutputFailure };
struct CouponReport { CouponStatus status; std::string diagnostic; };
struct ElasticCouponMetrics {
    tl::fea::NodalStamp stamp;
    tl::fea::reissner::ShellBatchDiagnostics diagnostics;
    double initial_energy = 0, maximum_relative_energy_error = 0;
    double last_operator_norm = 0;
    std::uint64_t last_operator_epoch = 0, required_steps = 0;
    std::uint64_t full_state_audit_reads = 0;
};
struct ElasticCouponFrame {
    tl::fea::NodalStamp stamp;
    std::array<double,3*reference::kCouponNodes> position{},velocity{},omega{},reaction_force{},reaction_couple{};
    std::array<double,4*reference::kCouponNodes> rotation{};
    std::array<tl::fea::reissner::ShellResult,reference::kCouponElements> element;
    tl::fea::reissner::ShellBatchDiagnostics element_association;
    ElasticCouponMetrics metrics;
};

// Composition of existing TL state/stepper, resident element batch and Chrono
// output. There is one dynamics clock. Reference/modal data are startup and
// declared diagnostic oracles only. Step reads small aggregate device results;
// complete positions/rotations are read only at specified spectral audit frames.
class ElasticCouponCase {
  public:
    ElasticCouponCase();
    ~ElasticCouponCase();
    ElasticCouponCase(const ElasticCouponCase&)=delete;
    ElasticCouponCase& operator=(const ElasticCouponCase&)=delete;
    CouponReport Initialize(const ElasticCouponConfig& = {});
    CouponReport Step(const ElasticCouponStepRequest& = {});
    // Explicit accepted output cadence. Caller output is staged before publish;
    // Stale result scratch is refreshed from accepted coordinates in a discarded
    // force-assembly trial. This cannot advance time or replace interval metrics.
    CouponReport Capture(ElasticCouponFrame&);
    const ElasticCouponMetrics* metrics() const noexcept;
    const reference::ElasticCouponModalReport* modal() const noexcept;
    const reference::ElasticCouponData* model_data() const noexcept;
    const visual::NodalMeshOutput* output() const noexcept;
    tl::fea::NodalAllocationInfo state_allocations() const noexcept;
    tl::fea::NodalAllocationInfo element_allocations() const noexcept;
  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
}  // namespace crash::case_data
