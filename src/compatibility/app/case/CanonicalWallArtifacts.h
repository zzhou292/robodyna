#pragma once

#include "CanonicalWall.h"
#include <filesystem>
#include <string>

namespace crash::case_data {
inline constexpr const char* kCanonicalWallManifestSha256 =
    "12500bf1512f1c06c9228cd319bf28d6087f62a40f294222f2fe38030b7d03c4";
// Bounded authenticated bytes are returned for the owning loader, avoiding a
// second file open between authentication and parse. All failures throw.
std::string ReadPinnedWallManifest(const std::string& path);
void CheckCanonicalWallBinding(const CanonicalWall&, const std::string& verified_bytes);
// Existing new directory; writes the original manifest and actual Chrono mesh
// JSON/OBJ once. Caller owns inventory and completed-manifest publication.
void WriteCanonicalWallArtifacts(const std::filesystem::path&, const CanonicalWall&,
                                 const std::string& verified_bytes);
} // namespace crash::case_data
