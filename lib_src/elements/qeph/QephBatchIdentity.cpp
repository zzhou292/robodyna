#include "QephBatchStorage.h"
#include "QephHistory.h"

namespace tl::fea::qeph::batch_detail {
bool SameDiagnostics(const BatchDiagnostics& a,const BatchDiagnostics& b) noexcept {
#define SAME_VALUE(field) if(a.field!=b.field) return false
#define SAME_DOUBLE(field) if(!detail::SameHistoryBits(a.field,b.field)) return false
  SAME_VALUE(owner_id); SAME_VALUE(configuration_id); SAME_VALUE(qualification_id);
  SAME_VALUE(epoch); SAME_VALUE(base_epoch); SAME_VALUE(attempt); SAME_VALUE(phase);
  SAME_VALUE(valid); SAME_VALUE(has_completed_interval); SAME_VALUE(accepted_force_assembled); SAME_VALUE(usage);
  SAME_DOUBLE(time); SAME_DOUBLE(base_time); SAME_DOUBLE(velocity_time); SAME_DOUBLE(base_velocity_time); SAME_DOUBLE(kick_dt);
  SAME_DOUBLE(kinetic_translation); SAME_DOUBLE(kinetic_rotation);
  SAME_DOUBLE(kinetic_physical_isotropic); SAME_DOUBLE(kinetic_added_isotropic);
  for(unsigned i=0;i<2;++i) { SAME_DOUBLE(internal_work[i]); SAME_DOUBLE(internal_work_increment[i]); }
  SAME_DOUBLE(hourglass_viscous_work); SAME_DOUBLE(hourglass_viscous_work_increment);
  SAME_DOUBLE(minimum_area_ratio); SAME_DOUBLE(minimum_thickness_ratio); SAME_DOUBLE(maximum_displacement);
  SAME_DOUBLE(maximum_absolute_strain); SAME_DOUBLE(maximum_thickness_curvature); SAME_DOUBLE(minimum_native_dt);
  SAME_DOUBLE(internal_kick_work); SAME_DOUBLE(internal_drift_work);
  return true;
#undef SAME_VALUE
#undef SAME_DOUBLE
}
bool SameStamp(const NodalStamp& a,const NodalStamp& b) noexcept {
  return a.owner_id==b.owner_id&&a.epoch==b.epoch&&a.node_count==b.node_count&&a.time==b.time&&
    a.fixed_dt==b.fixed_dt&&a.has_rotations==b.has_rotations&&a.reactions_valid==b.reactions_valid&&
    a.reaction_base_epoch==b.reaction_base_epoch&&a.reaction_time==b.reaction_time&&
    a.temporal_scheme==b.temporal_scheme&&a.velocity_phase==b.velocity_phase&&
    a.velocity_time==b.velocity_time&&a.reaction_kick_dt==b.reaction_kick_dt;
}
namespace {
bool SameView(const DeviceNodalKinematicsView& a,const DeviceNodalKinematicsView& b) {
  return a.position_xyz==b.position_xyz&&a.velocity_xyz==b.velocity_xyz&&
    a.angular_velocity_xyz==b.angular_velocity_xyz&&a.orientation_wxyz==b.orientation_wxyz&&
    a.node_count==b.node_count&&a.base_epoch==b.base_epoch;
}
}
bool SamePrepared(const NodalPreparedView& a,const NodalPreparedView& b) noexcept {
  return SameView(a.kinematics,b.kinematics)&&SameView(a.base_kinematics,b.base_kinematics)&&
    a.stream==b.stream&&a.owner_id==b.owner_id&&a.attempt==b.attempt&&a.proposed_time==b.proposed_time&&
    a.temporal_scheme==b.temporal_scheme&&a.velocity_phase==b.velocity_phase&&a.base_velocity_phase==b.base_velocity_phase&&
    a.base_time==b.base_time&&a.velocity_time==b.velocity_time&&a.base_velocity_time==b.base_velocity_time&&a.kick_dt==b.kick_dt;
}
bool ValidKinematics(const DeviceNodalKinematicsView& v,std::size_t n,std::uint64_t epoch) noexcept {
  return v.node_count==n&&v.base_epoch==epoch&&v.position_xyz&&v.velocity_xyz&&
    v.angular_velocity_xyz&&v.orientation_wxyz;
}
} // namespace tl::fea::qeph::batch_detail
