#pragma once
#include "NormalImpactCase.h"
#include "CanonicalWallArtifacts.h"
#include <cstddef>
#include <memory>
#include <string>

namespace crash::case_data {
// Output-only application module. A new directory is required. Existing Chrono
// OBJ is visualization precision; full-precision Chrono JSON archives are also
// emitted and read back to check coordinate bits/connectivity before inventory.
// The first epoch-zero frame binds owner/run/topology and nodal phase; subsequent
// records cannot mix other owners. failure.json is best-effort on failures after
// construction; absence of manifest.json always means output is incomplete.
// CSV force values belong to t_n and its completed interval; fields/metrics are
// accepted t_(n+1). No output operation owns dynamics, acceptance or histories.
class NormalImpactArtifacts {
  public:
    NormalImpactArtifacts(const std::string& new_directory, const std::string& canonical_bytes,
                          const CanonicalWall&, const NormalImpactConfig&, double horizon,
                          unsigned frame_every);
    ~NormalImpactArtifacts();
    NormalImpactArtifacts(const NormalImpactArtifacts&) = delete;
    NormalImpactArtifacts& operator=(const NormalImpactArtifacts&) = delete;
    void RecordInterval(const tl::fea::NodalStamp& begin, const ImpactMetrics& accepted,
                        const tlfea::contact::PlanarContactDiagnostics&);
    void WriteAcceptedFrame(const visual::NodalMeshOutput&, const ImpactMetrics&);
    void Finish(const NormalImpactCase&, double elapsed_seconds);
    void Fail(const char* message, const ImpactMetrics* last_accepted) noexcept;
  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
}  // namespace crash::case_data
