// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../qbat_mapped_gather/Fixture.h"
#include <limits>
namespace qbat_activity_test {
inline constexpr unsigned FaultCount=19;
inline void Corrupt(tl::fea::qbat::BatchResult& result,unsigned fault) {
  const auto nan=std::numeric_limits<double>::quiet_NaN();
  const unsigned char bad=2;
  switch(fault) {
    case 0:std::memcpy(&result.history.element_active,&bad,1);break;
    case 1:std::memcpy(&result.history.point[3].surface_active,&bad,1);break;
    case 2:std::memcpy(&result.point[3].failed_now,&bad,1);break;
    case 3:result.stamp.time+=1;break;
    case 4:++result.stamp.sample_index;break;
    case 5:result.history.thickness_m=-1;break;
    case 6:result.history.point[3].material.stress[4]=nan;break;
    case 7:result.history.point[3].failure.failure_time_s=result.stamp.time+1;break;
    case 8:result.history.force_stress_pa[0]=nan;break;
    case 9:result.kinematics.geometry.frame.v[8]=nan;break;
    case 10:result.kinematics.geometry.point[3].membrane_b_per_m[3]=nan;break;
    case 11:result.kinematics.corrected_velocity[3].z=nan;break;
    case 12:result.kinematics.equivalent_rate_per_s[3]=-1;break;
    case 13:result.diagnostics.rotation_stiffness_nm=1;break;
    case 14:result.diagnostics.internal_work_increment_j[1]=nan;break;
    case 15:result.point[3].material.equivalent_stress_pa=nan;break;
    case 16:result.point[3].force_volume_m3=nan;break;
    case 17:result.internal_force_n[3].z=nan;break;
    case 18:result.internal_couple_nm[3].y=1;break;
  }
}
} // namespace qbat_activity_test
