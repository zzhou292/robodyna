#pragma once
#include "Settings.h"
#include "case/shell_collection/ShellCollectionContactGeometry.h"
#include "case/PlacedCanonicalWall.h"
#include "case/vehicle_runtime/SourceIdentity.h"

namespace crash::cases::vehicle_wall {
struct Limits {
    std::size_t host_bytes=std::size_t{8}<<30;
    std::size_t wall_manifest_bytes=1u<<20;
    ShellContactGeometryLimits geometry=ShellContactGeometryLimits::Vehicle();
};
struct SetupForecast {
    ShellContactGeometryFootprint geometry;
    std::size_t shared_source_upper_bound=0,retained_setup_bytes=0,peak_host_upper_bound=0;
};
// Complete immutable source/wall setup. The authenticated placed original wall
// and its coverage assessment are always retained. The named generated profile
// has distinct feature IDs and explicit extents, with no source-geometry claim.
// No coefficient model, physical owner or accepted dynamics is created here.
class VehicleWallSetup {
  public:
    static SetupForecast Preflight(const vehicle_runtime::Execution&,const vehicle_runtime::Attachments&,
        const case_data::CanonicalWall&,const std::string& wall_bytes,const Settings& = {},Limits = {});
    static VehicleWallSetup Prepare(const vehicle_runtime::Execution&,const vehicle_runtime::Attachments&,
        const case_data::CanonicalWall&,const std::string& wall_bytes,const Settings& = {},Limits = {});
    const vehicle_runtime::Execution& execution() const noexcept;
    const vehicle_runtime::Attachments& attachments() const noexcept;
    const Settings& settings() const noexcept;
    const SetupForecast& forecast() const noexcept;
    const ShellCollectionContactGeometry& geometry() const noexcept;
    const case_data::PlacedCanonicalWall& wall() const noexcept;
    tlfea::contact::PlanarWallView selected_wall_view() const noexcept;
    const tlfea::contact::PlanarWallGeometry& selected_wall_geometry() const noexcept;
    const tlfea::contact::PlanarContactReport& original_coverage_report() const noexcept;
    const PlacementValues& placement() const noexcept;
    const tlfea::contact::PlanarWallBoxCoverage& coverage() const noexcept;
    const tlfea::contact::PlanarContactReport& coverage_report() const noexcept;
  private:
    struct Data;
    explicit VehicleWallSetup(std::shared_ptr<const Data> value):data_(std::move(value)) {}
    std::shared_ptr<const Data> data_;
};
} // namespace crash::cases::vehicle_wall
