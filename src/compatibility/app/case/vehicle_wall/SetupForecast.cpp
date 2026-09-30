#include "SetupData.h"
#include "case/CanonicalWallArtifacts.h"
#include "case/vehicle_runtime/Forecast.h"
#include "lib_utils/BoundedArena.h"
#include "output/ArtifactIO.h"

namespace crash::cases::vehicle_wall {
SetupForecast VehicleWallSetup::Preflight(const vehicle_runtime::Execution& execution,
    const vehicle_runtime::Attachments& attachments, const case_data::CanonicalWall& wall,
    const std::string& bytes, const Settings& settings, Limits limits) {
    output::Require(limits.host_bytes && limits.host_bytes <= std::size_t{20}*1000*1000*1000 &&
        limits.wall_manifest_bytes && limits.wall_manifest_bytes <= (1u<<20) && !bytes.empty() &&
        bytes.size() <= limits.wall_manifest_bytes, "Invalid wall setup host or manifest cap");
    CheckSettings(settings);
    vehicle_runtime::detail::CheckSource(execution,attachments);
    SetupForecast result;
    const auto geometry = ShellCollectionContactGeometry::ForecastMapped(
        *execution.physical().mapping(),result.geometry,limits.geometry);
    output::Require(bool(geometry),geometry.message);
    result.shared_source_upper_bound = vehicle_runtime::detail::SourceBytes(execution,attachments,limits.host_bytes);
    // SourceBytes retains the exact physical mapping; geometry's copy shares it.
    // Reserve complete original/generated wall geometry, bounded metadata and
    // both controls. The 1 MiB wall reserve covers the existing <=64/100 mesh
    // scratch/containers, generated 4/2 mesh and shared ownership overhead.
    tl::util::BoundedArenaLayout local(limits.host_bytes), complete(limits.host_bytes);
    tl::util::ArenaRegion unused;
    for (auto n : {sizeof(Data)+sizeof(VehicleWallSetup),result.geometry.retained_geometry_bytes,
                   2*bytes.size(),std::size_t{1}<<20}) {
        output::Require(local.Append<std::byte>(n,unused),"Wall retained setup exceeds host cap");
    }
    result.retained_setup_bytes = local.bytes();
    for (auto n : {result.shared_source_upper_bound,result.retained_setup_bytes,result.geometry.temporary_bytes}) {
        output::Require(complete.Append<std::byte>(n,unused),"Complete wall setup exceeds host cap");
    }
    result.peak_host_upper_bound = complete.bytes();
    case_data::CheckCanonicalWallBinding(wall,bytes);
    return result;
}
} // namespace crash::cases::vehicle_wall
