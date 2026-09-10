#include "SourceAssemblyWallSchema.h"
#include <limits>

namespace crash::output::assembly {
WallArchivePlan PlanWallArchive(const WallArchiveRequest& r,std::size_t source,std::size_t wall) {
    const auto& l=r.limits;
    Require(r.steps&&r.frame_every&&r.run_id&&r.topology_id&&r.asset_id&&
        l.total_bytes&&l.total_bytes<=WallArchiveTotalCap&&l.file_bytes>=WallFieldBytes&&
        l.file_bytes<=kArtifactFileCap&&l.frames&&l.frames<=kArtifactFrameCap&&
        source&&source<=4*1024*1024&&wall&&wall<=WallSmallFileBytes,"Invalid assembly archive request or limits");
    WallArchivePlan p;
    // Reference, regular cadence, final noncadence endpoint, and one explicitly
    // requested stopped-prefix endpoint. Extra frames cannot consume this reserve.
    const std::uint64_t regular=r.steps/r.frame_every;
    Require(regular<=l.frames,"Assembly frame forecast exceeds capacity");
    p.frame_capacity=1+static_cast<std::size_t>(regular)+(r.steps%r.frame_every!=0)+1;
    Require(p.frame_capacity<=l.frames,"Assembly frame forecast exceeds capacity");
    p.intervals=PlanCsvLedger("accepted-intervals.csv",WallIntervalHeader,r.steps,WallIntervalColumns*26,l.file_bytes);
    std::size_t bytes=0;
    auto add=[&](std::size_t n) {Require(n<=l.total_bytes-bytes,"Assembly archive forecast exceeds byte capacity");bytes+=n;};
    add(source);add(wall);add(WallConfigurationBytes);add(WallMeshBytes);add(WallObjBytes);
    add(WallSmallFileBytes);add(WallSmallFileBytes); // Placement and final metrics.
    add(WallFrameIndexBytes);add(WallManifestBytes);add(p.intervals.total_bytes);
    constexpr auto frame=WallFieldBytes+WallMeshBytes+WallObjBytes;
    Require(p.frame_capacity<=(l.total_bytes-bytes)/frame,"Assembly frame payload exceeds archive capacity");
    add(p.frame_capacity*frame);p.forecast_bytes=bytes;
    p.forecast_files=3*p.frame_capacity+9+p.intervals.segments.size(); // Includes final manifest.
    Require(p.forecast_files-1<=kArtifactInventoryCap,"Assembly inventory forecast exceeds capacity");
    return p;
}
} // namespace crash::output::assembly
