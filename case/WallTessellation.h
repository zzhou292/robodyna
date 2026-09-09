#pragma once

#include "CanonicalWall.h"
#include "collision/PlanarWallGeometry.h"
#include "collision/Q4PlanarGeometry.h"
#include <memory>

namespace crash::case_data {
enum class WallTessellationKind : std::uint8_t { Original=0, FlipConvexPairs=1, UniformFour=2 };
const char* WallTessellationName(WallTessellationKind) noexcept;
inline constexpr std::uint64_t kOriginalWallTessellationBinding=0x5941524953574131ULL;
// Fixed experiment bindings, not source node/element IDs. Invalid kinds return
// zero. Geometry/source authentication still requires the retained metadata.
std::uint64_t WallTessellationBindingId(WallTessellationKind) noexcept;
// These IDs are explicitly synthetic feature identifiers, not imported deck
// node/element IDs. A derived mesh is identified by source SHA + transform +
// mesh SHA, and requires a distinct owner wall binding in mechanical use.
inline constexpr std::uint64_t kWallMidpointIdBase=0x8000000000000000ULL;
inline constexpr std::uint64_t kWallDerivedFaceIdBase=0xc000000000000000ULL;

struct WallMidpointSource {
    std::uint32_t vertex=0;
    std::array<std::uint32_t,2> original_edge{};
    std::array<std::uint64_t,2> original_source_nodes{},original_assembled_nodes{};
    tlfea::contact::Q4IntegralInterval exact_midpoint_y{},exact_midpoint_z{};
    tlfea::contact::Vec3 coordinate_error; // Absolute m; X is exactly unchanged.
    bool exposed_edge=false;
};
struct WallFaceSource {
    std::array<std::uint64_t,2> original_triangle_ids{};
    std::uint32_t original_triangle_count=0,subtriangle=0;
    bool connectivity_changed=false;
};
struct WallTessellationMetadata {
    WallTessellationKind kind=WallTessellationKind::Original;
    std::string transform_version,source_manifest_sha256,mesh_sha256;
    std::uint32_t original_vertices=0,original_triangles=0,original_boundary_edges=0,derived_boundary_edges=0;
    std::vector<std::uint64_t> flipped_source_quads;
    std::vector<WallMidpointSource> midpoints;
    std::vector<WallFaceSource> faces; // Same order as view().triangles.
    double exposed_boundary_displacement_bound_m=0;
    bool exposed_boundary_exact=true;
};
enum class WallTessellationStatus { Ok,InvalidInput,NotInitialized,AlreadyInitialized,GeometryFailure,ResourceLimit };
struct WallTessellationReport { WallTessellationStatus status; std::string diagnostic; };

// Host startup geometry only. Authenticates original CanonicalWall bytes,
// deterministically transforms a copy and validates its WHOLE finite mesh.
// No CUDA runtime, physical state, force integration or dynamics clock. Original
// source provenance remains separate from explicitly synthetic derived IDs.
// Plane X is exact. Rounded diagonal-edge midpoints have outward coordinate
// bounds; their physical boundary is NOT claimed bitwise/mathematically exact.
// Immutable publication occurs once after all checks. Failure leaves output
// empty and preserves a previously initialized object. Calls serialize.
class WallTessellation {
  public:
    WallTessellation();
    ~WallTessellation();
    WallTessellation(const WallTessellation&)=delete;
    WallTessellation& operator=(const WallTessellation&)=delete;
    WallTessellationReport Initialize(const CanonicalWall&,const std::string& authenticated_bytes,WallTessellationKind);
    bool initialized() const noexcept;
    tlfea::contact::PlanarWallView view() const noexcept;
    const WallTessellationMetadata* metadata() const noexcept;
    const WallProvenance* source_provenance() const noexcept;
    const tlfea::contact::PlanarWallGeometry* geometry() const noexcept;
    // Reuse immutable original physical Q4 coordinates/mass/masks. All parents
    // must remain covered. Success publishes staged C3 reference; any failure
    // leaves caller output unchanged. No placement/search is performed.
    WallTessellationReport CheckCoverage(const tlfea::contact::Q4SurfaceView&,
        const tlfea::contact::Q4FixedYZMassView&,double clearance,tlfea::contact::Q4PlanarGeometry& output) const;
  private:
    struct Data;
    std::unique_ptr<const Data> data_;
};
} // namespace crash::case_data
