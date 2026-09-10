#pragma once
#include "Config.h"
#include "StepTiming.h"
#include "case/source_assembly_observation/SourceAssemblyObservation.h"
#include "case/source_assembly_observation/SourceAssemblyForceStageKinetic.h"
#include "case/source_assembly_observation/SourceAssemblyQephSpin.h"
#include <memory>

namespace crash::output::assembly { class SourceAssemblyAcceptedOutput; }
namespace crash::cases::source_assembly_dynamics {
struct Diagnostics {
    tl::fea::NodalStamp stamp;
    tl::fea::ShellBatchDiagnostics shells;
    source_assembly_observation::Summary motion;
    bool has_interval=false;
    double native_internal_work=0,maximum_rotation=0;
    double maximum_area_ratio=1,maximum_thickness_ratio=1;
    double maximum_plastic_strain=0,cumulative_plastic_work=0;
    std::size_t yielded_points=0,yielded_parents=0,active_contact_nodes=0;
    // Contact phase is the completed PreparedCandidate, associated with stamp.
    std::uint64_t first_contact_epoch=0,last_contact_epoch=0,contact_intervals=0;
};
struct ContactView {
    const tlfea::contact::NodalWallDiagnostics* diagnostics=nullptr;
    const tlfea::contact::NodalWallParentResult* parents=nullptr;
    const tlfea::contact::NodalWallPointResult* nodes=nullptr;
    const std::uint64_t* wall_face=nullptr;
    std::size_t parent_count=0,node_count=0;
};
// One app coordinator over the existing TL owner, grouped recurrence, two
// native material batches, contact contributor and joined publication. Calls
// serialize on the owner's stream. All candidate readback/observations precede
// the sole commit; host captures contain copied observations, not another
// mechanical state/clock. No per-step allocation. CUDA errors poison the run.
class SourceAssemblyWallCase {
  public:
    SourceAssemblyWallCase();
    ~SourceAssemblyWallCase();
    SourceAssemblyWallCase(const SourceAssemblyWallCase&)=delete;
    SourceAssemblyWallCase& operator=(const SourceAssemblyWallCase&)=delete;
    Report Initialize(const source_assembly::SourceAssemblyBindings&,
        const source_assembly::SourceAssemblyWallSetup&,const Config&,StepTimingOptions={});
    Report Step();
    // Copied diagnostic counters only; never an accepted physics observation.
    StepTimingSnapshot timing() const noexcept;
    Report CaptureAccepted(output::assembly::SourceAssemblyAcceptedOutput&);
    bool initialized() const noexcept;
    const tl::fea::FENodalState* owner() const noexcept;
    const source_assembly::SourceAssemblyBindings* bindings() const noexcept;
    const source_assembly::SourceAssemblyWallSetup* setup() const noexcept;
    const Config* config() const noexcept;
    const Diagnostics* diagnostics() const noexcept;
    // No contact candidate exists at epoch zero: this view is empty then.
    ContactView accepted_contact() const noexcept;
    // Empty initially/off. A completed force-stage sample becomes visible only
    // with its enclosing common commit; the sample itself is at base_time.
    const source_assembly_observation::ForceStageSummary* accepted_force_stage() const noexcept;
    // Actual base-time packets/loads, exposed only with their enclosing commit.
    const source_assembly_observation::QephSpinObservation* accepted_qeph_spin() const noexcept;
    tl::fea::NodalAllocationInfo allocations() const noexcept;
    std::size_t host_payload_bytes() const noexcept;
  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
    friend struct SourceAssemblyDynamicsTestAccess;
};
} // namespace crash::cases::source_assembly_dynamics
