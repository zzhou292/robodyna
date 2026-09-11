#pragma once
#include "Settings.h"
#include <array>
namespace crash::cases::vehicle_wall {
// A distinct generated mesh, never a transformed original-source wall. Its
// high-bit feature namespace contains no imported source IDs.
class EnvelopeWall {
  public:
    static EnvelopeWall Prepare(const PlacementValues&, const Settings&);
    tlfea::contact::PlanarWallView view() const noexcept;
    const tlfea::contact::PlanarWallGeometry& geometry() const noexcept { return geometry_; }
    const tlfea::contact::PlanarWallBox& extent() const noexcept { return extent_; }
  private:
    std::array<tlfea::contact::PlanarWallVertex,4> vertices_{};
    std::array<tlfea::contact::PlanarWallTriangle,2> triangles_{};
    tlfea::contact::PlanarWallGeometry geometry_;
    tlfea::contact::PlanarWallBox extent_;
};
} // namespace crash::cases::vehicle_wall
