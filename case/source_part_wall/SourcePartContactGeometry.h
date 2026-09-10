#pragma once
#include "qualification/source_contact/SourcePartContactFixture.h"
#include "collision/NodalWallContact.h"
#include "collision/PlanarWallBox.h"
#include <array>
#include <memory>

namespace crash::cases::source_part_wall {
namespace contact=tlfea::contact;
namespace source=qualification::source_contact;

// Immutable original midsurface measures and contact area shares only. Native
// shell mass/inertia belongs to ShellBatchBinding and the eventual state owner;
// contact area shares are never converted into a mechanics mass policy here.
class SourcePartContactGeometry {
  public:
    SourcePartContactGeometry();
    ~SourcePartContactGeometry();
    SourcePartContactGeometry(const SourcePartContactGeometry&)=delete;
    SourcePartContactGeometry& operator=(const SourcePartContactGeometry&)=delete;
    bool Initialize(const source::SourcePartContactFixture&,std::string& diagnostic);
    bool prepared() const noexcept;
    const contact::NodalWallWeights* weights() const noexcept;
    std::array<contact::Vec3,2> reference_bounds() const noexcept;
    // Select by original source-parent order; the common weight owner retains
    // its own deterministic sorted parent order. Failure preserves output.
    bool ParentWeights(unsigned source_parent,contact::NodalWallWeights*) const;
    // Explicit inverse maps; invalid/unprepared queries return UINT32_MAX.
    unsigned weight_index(unsigned source_parent) const noexcept;
    unsigned source_parent_index(unsigned weight_index) const noexcept;
    // Setup coverage of one caller-declared WORLD motion envelope containing
    // the entire original part. Project its Y/Z bounds onto the actual mesh
    // plane: BOTH X endpoints equal wall.wall_x(), never the vehicle X range.
    // Existing finite-mesh exposed-edge/hole checks determine admission. This
    // neither proves a future trajectory remains in the envelope nor applies
    // forces. Failure preserves every output field.
    contact::PlanarContactReport CheckWallCoverage(const contact::PlanarWallGeometry&,
        contact::Vec3 motion_minimum,contact::Vec3 motion_maximum,double exposed_clearance,
        std::uint64_t feature_id,contact::PlanarWallBoxCoverage*) const;
  private:
    struct Data;
    std::unique_ptr<const Data> data_;
};
} // namespace crash::cases::source_part_wall
