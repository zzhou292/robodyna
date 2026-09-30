#pragma once

#include "WallTessellation.h"
#include <filesystem>

namespace crash::case_data {
inline constexpr std::size_t kWallStudyProvenanceByteCap=1024*1024;
struct VerifiedWallStudyProvenance {
    WallTessellationKind kind=WallTessellationKind::Original;
    std::uint64_t owner_id=0,wall_binding_id=0;
    std::string study_sha256,source_manifest_sha256,derived_mesh_sha256;
    std::uint32_t derived_vertices=0,derived_triangles=0;
};
// Sidecar only: not a canonical guided-v1 replay bundle, restart or independent
// qualification of mechanics. Binds exact completed Study JSON bytes to the
// authenticated original wall and explicit deterministic transform lineage.
// Creation requires an absent destination and validates before writing.
void WriteWallStudyProvenance(const std::filesystem::path&,const CanonicalWall&,
    const std::string& canonical_bytes,const WallTessellation&,const std::string& exact_study_bytes);
// Both read paths regenerate the wall from supplied authenticated source,
// compare every required metadata/lineage field exactly and check the fixed
// variant wall-binding identity and stored physical footprint coverage.
// Required duplicates reject; unknown extension fields are ignored. All input
// bytes are bounded and parsed in full precision. Failure throws before a
// returned verified result exists; no caller state or geometry is modified.
VerifiedWallStudyProvenance ParseWallStudyProvenance(const std::string& sidecar_bytes,const CanonicalWall&,
    const std::string& canonical_bytes,const std::string& exact_study_bytes);
VerifiedWallStudyProvenance ReadWallStudyProvenance(const std::filesystem::path&,const CanonicalWall&,
    const std::string& canonical_bytes,const std::string& exact_study_bytes);
} // namespace crash::case_data
