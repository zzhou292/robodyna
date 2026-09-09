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
} // namespace tl::fea::qeph::batch_detail
