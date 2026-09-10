#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <string>

namespace chrono { class ChTriangleMeshConnected; }

namespace crash::output {

enum class ReplayKind { NormalImpact, ElasticCoupon, GuidedPlate, SourcePartElastic, SourcePartWall };
enum class ReplayStatus { Ok, InvalidBundle, NotInitialized, InvalidFrame };
struct ReplayReport { ReplayStatus status; std::string diagnostic; };
struct ReplayInfo {
    ReplayKind kind = ReplayKind::NormalImpact;
    std::string schema, scope;
    std::uint64_t owner_id = 0, final_epoch = 0;
    double final_time = 0;
    std::size_t frame_count = 0, node_count = 0, triangle_count = 0;
    // Moving-surface bounds over every verified frame, excluding the wall.
    std::array<double, 3> bounds_min{}, bounds_max{};
    // Present only when explicitly archived (the shell schemas). Old normal
    // impact meshes have no synthetic replacement run/topology/source IDs.
    std::uint64_t run_id = 0, topology_id = 0;
    // Validated guided name; empty for other replay kinds. Missing legacy
    // guided metadata resolves to the original experiment only.
    std::string guided_experiment;
    bool horizon_complete=true; // Explicit wall accepted prefixes may stop early.
    std::string stop_reason;
};
struct ReplayFrame {
    std::size_t index = 0;
    std::uint64_t owner_id = 0, epoch = 0;
    double time = 0;
    std::shared_ptr<const chrono::ChTriangleMeshConnected> mesh;
};

// Validated, accepted-only geometry replay; no solver, physical clock or state
// owner. Open checks the completed manifest, every inventoried hash/byte count,
// index provenance and every mesh before publishing frame zero. Load rechecks
// requested file bytes before publishing one frame. Failures preserve the last
// complete reader state. Caller serializes calls and must not mutate the bundle.
// Geometry/record association is checked; this does not requalify the physical
// interval ledger or authenticate original deck source ownership. Normal-impact
// bundles require their archived canonical wall; coupon bundles have no wall.
// Guided-plate bundles additionally bind complete endpoint fields and the exact
// pinned original canonical wall. Derived wall variants use a separate contract.
//
// Preview caps: 1000 frames, 4096 vertices/8192 triangles per mesh, 32 MiB per
// file and 256 MiB declared inventory. Only metadata, immutable connectivity,
// the current mesh, optional wall and temporary staging are retained. Callers
// should release old frame.mesh handles rather than accumulating a whole run.
class AcceptedReplay {
  public:
    AcceptedReplay();
    ~AcceptedReplay();
    AcceptedReplay(const AcceptedReplay&) = delete;
    AcceptedReplay& operator=(const AcceptedReplay&) = delete;
    ReplayReport Open(const std::filesystem::path& directory);
    ReplayReport Load(std::size_t frame_index);
    const ReplayInfo* info() const noexcept;
    const ReplayFrame* frame() const noexcept;
    std::shared_ptr<const chrono::ChTriangleMeshConnected> wall() const noexcept;
  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace crash::output
