#pragma once

#include "CanonicalWall.h"
#include "WallTessellation.h"
#include "GuidedPlateAdmission.h"
#include "chrono/GuidedPlateExperiment.h"
#include "chrono/GuidedPlatePenaltyAudit.h"
#include "chrono/NodalMeshOutput.h"
#include <memory>

namespace crash::case_data {
inline constexpr std::uint64_t kGuidedPlateQualification = reference::kGuidedOriginalExperiment.qualification_id;
inline constexpr std::uint64_t kGuidedPlateWallBinding = kOriginalWallTessellationBinding;
inline constexpr std::size_t kGuidedPlateDeviceBudget = 1024 * 1024;

struct GuidedPlateConfig {
    unsigned refinement = 1; // Independent fixed-step runs: 1, 2 or 4.
    unsigned diagnostic_intervals = 20;
    tlfea::contact::Q4PlanarIntegrationBackend integration_backend =
        tlfea::contact::Q4PlanarIntegrationBackend::ScalarDyadicSquares;
    reference::GuidedPlateExperiment experiment = reference::GuidedPlateExperiment::Original;
};
struct GuidedPlateStepRequest {
    // Tightening this attempt's stop envelope cannot change the experiment.
    double maximum_displacement = ElasticShellLimits::displacement;
};
enum class GuidedPlateStatus {
    Ok, InvalidInput, NotInitialized, AlreadyInitialized, StateFailure,
    ElementFailure, ContactFailure, AdmissionFailure, AuditFailure, OutputFailure
};
struct GuidedPlateReport { GuidedPlateStatus status; std::string diagnostic; };
struct GuidedPlateMetrics {
    tl::fea::NodalStamp stamp;
    // Initial accepted-base diagnostics, then matching accepted ENDPOINT
    // candidate diagnostics. Their base_epoch still names the consumed interval.
    tl::fea::reissner::ShellBatchDiagnostics shell;
    tlfea::contact::Q4PlanarContactDiagnostics contact;
    // Force/reaction actually applied over the last accepted interval, evaluated
    // at its BASE. This is distinct from the endpoint contact above.
    tlfea::contact::Q4PlanarContactDiagnostics applied_contact;
    GuidedPlateWorkReport work;
    double initial_energy = 0, maximum_relative_energy_error = 0, peak_penetration = 0;
    tlfea::contact::Vec3 wall_impulse, wall_moment_impulse;
    double shell_midpoint_work = 0, contact_midpoint_work = 0;
    double shell_coordinate_work = 0, contact_coordinate_work = 0;
    double last_operator_norm = 0;
    std::uint64_t last_operator_epoch = 0, required_steps = 0, full_state_audit_reads = 0;
};
struct GuidedPlateFrame {
    tl::fea::NodalStamp stamp;
    std::array<double,3*reference::kCouponNodes> position{},velocity{},omega{},reaction_force{},reaction_couple{};
    std::array<double,4*reference::kCouponNodes> rotation{};
    std::array<tl::fea::reissner::ShellResult,reference::kCouponElements> element;
    std::array<tlfea::contact::Q4PlanarParentResult,reference::kCouponElements> parent;
    tl::fea::reissner::ShellBatchDiagnostics element_association;
    tlfea::contact::Q4PlanarContactDiagnostics contact_association;
    GuidedPlateMetrics metrics;
};

// A small composition of the qualified model, TL nodal owner, shell/contact
// contributors, complete case admission and accepted Chrono output. No second
// physical state or clock. Step transfers scalar diagnostics; full x/q transfer
// is confined to declared structural-audit cadence. Calls serialize externally.
// No force, mass, contact law or numerical admission is implemented here.
class GuidedPlateCase {
  public:
    GuidedPlateCase();
    ~GuidedPlateCase();
    GuidedPlateCase(const GuidedPlateCase&)=delete;
    GuidedPlateCase& operator=(const GuidedPlateCase&)=delete;
    GuidedPlateReport Initialize(const CanonicalWall&,const GuidedPlateConfig& = {});
    // Authenticated, explicit wall-only experiment variant. Plate setup/pose,
    // modal audit and initial state always use the ORIGINAL canonical wall.
    // Runtime coverage/area/stiffness must agree exactly before state startup.
    // Derived variants are not admitted by the canonical artifact writer.
    GuidedPlateReport InitializeTessellated(const CanonicalWall&,const std::string& authenticated_source_bytes,
                                           WallTessellationKind,const GuidedPlateConfig& = {});
    GuidedPlateReport Step(const GuidedPlateStepRequest& = {});
    // Both contributor results are staged before accepted publication. Stale
    // scratch is refreshed by assembling BOTH at accepted state and discarding
    // that temporary trial. Interval metrics, reactions and time never change.
    GuidedPlateReport Capture(GuidedPlateFrame&);
    const GuidedPlateMetrics* metrics() const noexcept;
    // Immutable execution choice. Before successful initialization this returns
    // the scalar default; metrics() must be nonnull to identify a live case.
    tlfea::contact::Q4PlanarIntegrationBackend integration_backend() const noexcept;
    reference::GuidedPlateExperiment experiment() const noexcept;
    const reference::GuidedPlatePenaltyReport* penalty_audit() const noexcept;
    const reference::GuidedPlateModalReport* modal() const noexcept;
    const reference::ElasticCouponData* model_data() const noexcept;
    const reference::GuidedPlateData* guided_data() const noexcept;
    // Immutable host preparation, including exact-coordinate area bounds.
    tlfea::contact::Q4PlanarReferenceView contact_reference() const noexcept;
    std::uint64_t diagnostic_stride() const noexcept;
    const visual::NodalMeshOutput* output() const noexcept;
    tlfea::contact::PlanarWallView wall_mesh() const noexcept;
    const WallProvenance* wall_provenance() const noexcept;
    // Null for the legacy Initialize path; otherwise actual immutable runtime
    // transform/source/mesh metadata, separate from original source provenance.
    const WallTessellationMetadata* wall_tessellation() const noexcept;
    tl::fea::NodalAllocationInfo state_allocations() const noexcept;
    tl::fea::NodalAllocationInfo element_allocations() const noexcept;
    tl::fea::NodalAllocationInfo contact_allocations() const noexcept;
  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
    GuidedPlateReport InitializeImpl(const CanonicalWall&,const GuidedPlateConfig&,std::unique_ptr<WallTessellation>);
};
} // namespace crash::case_data
