#pragma once
#include "WallTessellation.h"
#include <filesystem>
#include <memory>

namespace crash::case_data {
struct WallPlacement {
    std::string source_manifest_sha256,source_mesh_sha256;
    double translation_x_m=0; // Declared applied operation, retained bitwise.
    double wall_x_m=0;        // Represented .05 + translation_x_m result.
};
enum class PlacedWallStatus { Ok,InvalidInput,AlreadyInitialized,GeometryFailure,ResourceLimit };
struct PlacedWallReport { PlacedWallStatus status; std::string diagnostic; };

// Host-only immutable X placement of the complete authenticated original wall.
// No remeshing, new source IDs, mass, force law, collision response or clock.
// The canonical loader and its exact X=.05 contract remain unchanged. All
// vertices/triangles are copied, with original Y/Z/connectivity/IDs preserved.
// Initial failure leaves this object empty; every later call preserves it.
class PlacedCanonicalWall {
  public:
    PlacedCanonicalWall();
    ~PlacedCanonicalWall();
    PlacedCanonicalWall(const PlacedCanonicalWall&)=delete;
    PlacedCanonicalWall& operator=(const PlacedCanonicalWall&)=delete;
    PlacedWallReport Initialize(const CanonicalWall&,const std::string& authenticated_bytes,double translation_x_m);
    bool initialized() const noexcept;
    tlfea::contact::PlanarWallView view() const noexcept;
    const tlfea::contact::PlanarWallGeometry* geometry() const noexcept;
    const WallPlacement* placement() const noexcept;
    const WallProvenance* source_provenance() const noexcept;
    const std::string* source_manifest() const noexcept;
  private:
    struct Data;
    std::unique_ptr<const Data> data_;
};

// Existing caller-owned new directory. Writes actual placed Chrono mesh/OBJ,
// original canonical manifest and separate source/transform/placed-mesh hashes.
// This does not write a completed run manifest or advance mechanics. File
// creation is checked/create-only by the shared artifact utilities; failure may
// leave partial files, which the owning case must inventory/fail accordingly.
void WritePlacedCanonicalWallArtifacts(const std::filesystem::path&,const PlacedCanonicalWall&);
} // namespace crash::case_data
