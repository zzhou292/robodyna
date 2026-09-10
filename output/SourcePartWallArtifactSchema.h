#pragma once
#include <cstddef>
namespace crash::output {
// Fixed 117-node/94-parent wall schema, including full per-node/parent
// certificates. Replacing every number by 26 bytes and bool by 5 bounds the
// dynamic field document at 476443 bytes and the mesh at 37798 bytes.
// OBJ has 117 vertex rows <=83 bytes and 182 face rows <=14 bytes: 12259 bytes.
inline constexpr std::size_t SourcePartWallFieldCap=480*1024;
inline constexpr std::size_t SourcePartWallMeshCap=40*1024;
inline constexpr std::size_t SourcePartWallObjCap=16*1024;
inline constexpr std::size_t SourcePartWallFrameCap=
    SourcePartWallFieldCap+SourcePartWallMeshCap+SourcePartWallObjCap;
inline constexpr const char* SourcePartWallIntervalHeader="owner_id,base_epoch,attempt,base_time_s,accepted_epoch,accepted_time_s,velocity_time_s,kick_dt_s,synchronized_kinetic_J,native_internal_work_J,energy_residual_J,kinetic_work_residual_J,kinetic_work_allowance_J,contact_resultant_N,contact_resultant_error_N,contact_potential_J,contact_potential_error_J,contact_kick_work_J,contact_drift_work_J,contact_work_uncertainty_J,contact_quadratic_work_upper_J,wall_kick_impulse_N_s,wall_kick_impulse_error_N_s,cumulative_wall_kick_impulse_N_s,cumulative_wall_kick_impulse_error_N_s,maximum_penetration_m,active_nodes,synchronized_kinetic_uncertainty_J,energy_allowance_J,momentum_residual_x_kg_m_per_s,momentum_allowance_x_kg_m_per_s,maximum_rotation_rad,physical_energy_uncertainty_J,strictly_separated_nodes\n";
inline constexpr unsigned SourcePartWallIntervalColumns=34;
}
