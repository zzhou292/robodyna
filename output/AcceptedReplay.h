#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

namespace chrono { class ChTriangleMeshConnected; }

namespace crash::output {

enum class ReplayKind { NormalImpact, ElasticCoupon, GuidedPlate, SourcePartElastic, SourcePartWall, SourceAssemblyWall };
enum class ReplayStatus { Ok, InvalidBundle, NotInitialized, InvalidFrame };
struct ReplayReport { ReplayStatus status; std::string diagnostic; };
struct ReplayAssemblyInfo {
    std::uint64_t source_instance_id=0;
    std::string inventory_sha256,boundary_policy;
    std::size_t inventory_bytes=0,parents=0,qeph=0,t3=0,groups=0,members=0;
    bool observe_force_stage=false;
    std::vector<std::uint64_t> part_ids,material_ids,section_ids,curve_ids;
};
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
    bool source_plasticity=false;
    std::string material_model, material_policy; // Single-part schemas only.
    std::shared_ptr<const ReplayAssemblyInfo> source_assembly;
    // Display metadata only. Plastic wall replay keeps one fixed color scale
    // for the complete accepted prefix; values are dimensionless, not percent.
    double plastic_strain_color_max = 0;
    double source_initial_speed_m_per_s = 0;
    std::vector<std::uint64_t> triangle_source_parent;
    // Complete original PID per display triangle, copied only from validated
    // source bindings. Empty for schemas without original part provenance.
    std::vector<std::uint64_t> triangle_source_part;
};
enum class ReplayScalarApplicability { NativeValue, NotApplicable, Unavailable };
struct ReplayParentScalar {
    std::uint64_t source_parent = 0;
    double value = 0;
    // Legacy values default to native. Non-native tags use an uninterpreted
    // zero storage marker; it must never be rendered as zero plastic strain.
    ReplayScalarApplicability applicability = ReplayScalarApplicability::NativeValue;
};
struct ReplayFrame {
    std::size_t index = 0;
    std::uint64_t owner_id = 0, epoch = 0;
    double time = 0;
    std::shared_ptr<const chrono::ChTriangleMeshConnected> mesh;
    // Maximum equivalent plastic strain over applicable stored native points
    // (three layers in legacy source wall schemas). Explicit non-native tags
    // are not zero strain. No interpolation between parents or triangles.
    std::vector<ReplayParentScalar> parent_plastic_strain;
};

// Validated, accepted-only geometry replay; no solver, physical clock or state
// owner. Open checks the completed manifest, every inventoried hash/byte count,
// index provenance and every mesh before publishing frame zero. Load rechecks
// requested file bytes before publishing one frame. Failures preserve the last
// complete reader state. Caller serializes calls and must not mutate the bundle.
// Geometry/record association is checked; this does not requalify the physical
// interval mechanics. Assembly replay additionally authenticates the pinned original
// inventory and its complete source declarations; ownership is not inferred. Normal-impact
// bundles require their archived canonical wall; coupon bundles have no wall.
// Guided-plate bundles additionally bind complete endpoint fields and the exact
// pinned original canonical wall. Derived wall variants use a separate contract.
//
// Preview caps: 1000 frames, 4096 vertices/8192 triangles per mesh, 32 MiB per
// file and 256 MiB declared inventory (1 GiB for explicit plastic wall v2 or assembly wall v1). Only metadata, immutable connectivity,
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
