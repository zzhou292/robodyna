#pragma once
#include "output/CsvLedgerSegments.h"

namespace crash::output::assembly {
inline constexpr const char* WallArtifactSchema="robo_dyna.source_assembly_wall_artifacts.v1";
inline constexpr const char* WallConfigurationSchema="robo_dyna.source_assembly_wall_configuration.v1";
inline constexpr const char* WallFrameSchema="robo_dyna.source_assembly_wall_frame.v1";
inline constexpr const char* WallArtifactKind="source_assembly_wall";
inline constexpr std::size_t WallFieldBytes=8*1024*1024,WallMeshBytes=1024*1024,WallObjBytes=256*1024;
inline constexpr std::size_t WallConfigurationBytes=8*1024*1024,WallManifestBytes=1024*1024;
inline constexpr std::size_t WallSmallFileBytes=1024*1024,WallFrameIndexBytes=256*1024;
struct WallArchiveLimits {
    std::size_t total_bytes=kArtifactExtendedTotalCap,file_bytes=kArtifactFileCap,frames=kArtifactFrameCap;
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
inline constexpr const char* WallIntervalHeader=
    "owner_id,base_epoch,attempt,base_time_s,accepted_epoch,accepted_time_s,velocity_time_s,kick_dt_s,"
    "native_stored_kinetic_J,effective_stored_kinetic_J,native_internal_work_J,cumulative_plastic_work_J,"
    "native_kick_delta_J,effective_stored_delta_J,replacement_delta_J,applied_kick_work_J,reaction_kick_work_J,"
    "native_recurrence_residual_J,effective_bookkeeping_residual_J,roundoff_budget_J,"
    "wall_resultant_N,wall_resultant_error_N,wall_potential_J,wall_potential_error_J,"
    "wall_kick_work_J,wall_drift_work_J,wall_work_uncertainty_J,wall_quadratic_work_upper_J,"
    "wall_conservative_defect_J,wall_kick_impulse_N_s,wall_kick_impulse_error_N_s,"
    "maximum_penetration_m,active_contact_nodes,maximum_plastic_strain,yielded_points,yielded_parents,"
    "maximum_rotation_rad,maximum_area_ratio,maximum_thickness_ratio\n";
inline constexpr std::size_t WallIntervalColumns=39;
// Pure forecast. Checked arithmetic and exact segmented-ledger capacity precede
// directory creation, accepted-output capture and all output-side mutation.
WallArchivePlan PlanWallArchive(const WallArchiveRequest&,std::size_t source_bytes,std::size_t wall_bytes);
} // namespace crash::output::assembly
