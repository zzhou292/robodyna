// SPDX-License-Identifier: MIT
#pragma once
#include "../qbat_resident/Fixture.h"
#include "../qbat_resident/ResultValues.h"
namespace qbat_private_test {
namespace qb=tl::fea::qbat;
namespace batch=qb::batch_detail;
using qbat_force_test::Fixture;
using qbat_force_test::Path;
using qbat_resident_test::Element;
using qbat_resident_test::ResultValues;
inline void Dirty(qb::BatchResult& value) {
  value.stamp={123,456};
  value.history.reported_rate_per_s=789;
  for(unsigned i=0;i<4;++i) {
    value.kinematics.strain_increment[i][7]=123;
    value.point[i].failed_now=true;
    value.point[i].material.history.stress[3]=456;
    value.internal_couple_nm[i]={1,2,3};
  }
  value.diagnostics.rotation_stiffness_nm=999;
  value.diagnostics.removed_now=true;
}
}
