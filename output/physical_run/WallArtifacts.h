#pragma once
#include "Types.h"
#include "output/full_shell/static_bundle/Types.h"
#include <array>
namespace chrono { class ChTriangleMeshConnected; }
namespace crash::output::physical_run {
inline constexpr const char* WallProfile="vehicle_wall_setup_v1";
inline constexpr std::size_t WallFileCap=1u<<20;
inline constexpr std::size_t WallWorkspaceBytes=16u<<20;
inline constexpr std::size_t WallMeshRetainedBytes=512u<<10;
inline constexpr std::array<const char*,7> WallFiles{{
    "wall/original-canonical-wall.manifest.json","wall/placed-wall.mesh.json","wall/placed-wall.obj",
    "wall/placed-wall-placement.json","wall/selected-wall.mesh.json","wall/selected-wall.obj",
    "wall/vehicle-wall-setup.json"}};
struct WallReceipt {
    std::uint64_t source_instance_id=0,wall_binding_id=0;
    std::string source_mapping_sha256;
    std::array<records::RecordFile,7> files;
};
Document WallDocument(const WallReceipt&);
WallReceipt ReadWallDocument(const Value&);
// All hashes and named source associations are checked before Chrono allocation.
std::shared_ptr<const chrono::ChTriangleMeshConnected> ReadWallArtifacts(const std::filesystem::path&,
    const WallReceipt&,const records::source::CanonicalData&,const records::Context&);
} // namespace crash::output::physical_run
