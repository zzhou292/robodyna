// SPDX-License-Identifier: AGPL-3.0-or-later
#include "lib_src/collision/RadiossType25Coefficients.h"
#ifdef __FAST_MATH__
#error Qualified coefficient consumer cannot enable fast math
#endif
int main() {
  namespace n=tlfea::contact::radioss_type25;
  n::NativeSolidNodalShares solid;
  if(n::EvaluateNativeSolidNodalShares({n::SolidNodalKind::Penta6,6,1,5},&solid)!=n::CoefficientStatus::Ok)return 1;
  if(solid.defined_raw_slot_mask!=0x77||solid.volume_share!=1||solid.bulk_volume_share!=5)return 2;
  n::NativeSpringNodalInput in{n::SpringNodalKind::Type13,1,0,{{2,3},{3,4},{4,5}},0};
  n::NativeScalarCoefficient spring;
  if(n::EvaluateNativeSpringNodalCoefficient(in,&spring)!=n::CoefficientStatus::Ok||spring.value!=20)return 3;
  return 0;
}
