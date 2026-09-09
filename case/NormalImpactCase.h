#pragma once

#include "CanonicalWall.h"
#include "chrono/NodalMeshOutput.h"
#include "lib_src/collision/PlanarMeshContact.h"
#include <memory>

namespace crash::case_data {
struct NormalImpactConfig {
    double dt = .0005, areal_density = 100, penalty_per_area = 1e6;
    double speed = 1, max_penetration = .011;
    double center_y = 0, center_z = 1.302, width = .1;
    unsigned patch_divisions = 1;  // 1, 2 or 4: at most 25 nodes / 32 triangles.
    bool refine_wall = false;     // Conforming midpoint split of each wall triangle.
};
enum class ImpactStatus { Ok, InvalidInput, NotInitialized, AlreadyInitialized,
                          StateFailure, ContactFailure, ReadbackFailure, InvalidOutput };
struct ImpactReport { ImpactStatus status; const char* message; };
struct ImpactMetrics {
    tl::fea::NodalStamp stamp;
    double mass = 0, mean_gap = 0, mean_normal_velocity = 0;
    double kinetic_energy = 0, elastic_energy = 0;
    double wall_impulse = 0, contact_work = 0, peak_penetration = 0;
};

// A bounded contact/inertia integration case, not a shell or vehicle model.
// TL owns all dynamics and accepted time; this coordinator owns case assembly,
// precommit admission, interval bookkeeping and Chrono output cadence. The wall
// is finite canonical geometry; reference-area centroid forces act on physical
// triangle nodes. Only fixed-footprint normal motion is admitted by the batch.
// Startup allocates all owned CPU/device buffers. Calls are externally serialized.
class NormalImpactCase {
  public:
    NormalImpactCase();
    ~NormalImpactCase();
    NormalImpactCase(const NormalImpactCase&) = delete;
    NormalImpactCase& operator=(const NormalImpactCase&) = delete;
    ImpactReport Initialize(const CanonicalWall&, const NormalImpactConfig& = {});
    // Failure before Commit preserves metrics, previous interval and visible
    // mesh. No automatic time adaptation, retry, damping or mass scaling.
    ImpactReport Step();
    visual::Report Publish();  // Separate accepted-only output operation.
    const ImpactMetrics* metrics() const noexcept;
    // Forces evaluated at base_epoch n and applied over [t_n,t_(n+1)]. Valid
    // only after the corresponding commit. Never instantaneous end-step force.
    const tlfea::contact::PlanarContactDiagnostics* last_interval() const noexcept;
    const visual::NodalMeshOutput* output() const noexcept;
    tl::fea::NodalAllocationInfo state_allocations() const noexcept;
    tlfea::contact::PlanarContactAllocationInfo contact_allocations() const noexcept;
  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
}  // namespace crash::case_data
