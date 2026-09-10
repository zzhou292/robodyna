#pragma once
#include "output/CsvLedgerSegments.h"
#include "output/full_shell/IntervalValues.h"

namespace crash::output::assembly {
inline constexpr const char* WallArtifactSchema="robo_dyna.source_assembly_wall_artifacts.v1";
inline constexpr const char* WallConfigurationSchema="robo_dyna.source_assembly_wall_configuration.v1";
inline constexpr const char* WallFrameSchema="robo_dyna.source_assembly_wall_frame.v1";
inline constexpr const char* WallArtifactKind="source_assembly_wall";
// This archive version has curve-only material metadata. Analytic hardening
// declarations require their own complete output/reader qualification.
inline void CheckWallSourceSchema(const std::string& schema) {
    Require(schema=="robo-dyna.source-assembly-inventory.v1",
        "Legacy assembly wall archives require the V1 source inventory schema");
}
inline constexpr std::size_t WallFieldBytes=8*1024*1024,WallMeshBytes=1024*1024,WallObjBytes=256*1024;
inline constexpr std::size_t WallConfigurationBytes=8*1024*1024,WallManifestBytes=1024*1024;
inline constexpr std::size_t WallSmallFileBytes=1024*1024,WallFrameIndexBytes=256*1024;
inline constexpr std::size_t WallArchiveTotalCap=kArtifactMaximumTotalCap;
struct WallArchiveLimits {
    std::size_t total_bytes=WallArchiveTotalCap,file_bytes=kArtifactFileCap,frames=kArtifactFrameCap;
};
struct WallArchiveRequest {
    std::uint64_t steps=0,run_id=0,topology_id=0,asset_id=0;
    unsigned frame_every=0;
    WallArchiveLimits limits;
};
struct WallArchivePlan {
    CsvLedgerPlan intervals;
    std::size_t frame_capacity=0,forecast_bytes=0,forecast_files=0;
};
inline constexpr const char* WallIntervalHeader=interval::CsvHeader;
inline constexpr std::size_t WallIntervalColumns=interval::ColumnCount;
// Pure forecast. Checked arithmetic and exact segmented-ledger capacity precede
// directory creation, accepted-output capture and all output-side mutation.
WallArchivePlan PlanWallArchive(const WallArchiveRequest&,std::size_t source_bytes,std::size_t wall_bytes);
} // namespace crash::output::assembly
