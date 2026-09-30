// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Type13ModelInternal.h"

namespace tl::fea::type13::model_detail {
bool Same(const ModelPropertyInput& a,const ModelPropertyInput& b) noexcept {
  const auto& x=a.input;const auto& y=b.input;
  if(a.source_id!=b.source_id||!Same(x.units,y.units)||!Same(x.mass_per_length,y.mass_per_length)||
     !Same(x.inertia_per_length,y.inertia_per_length)||
     x.controls.length_normalized!=y.controls.length_normalized||
     x.controls.coupled_failure!=y.controls.coupled_failure||x.controls.force_failure!=y.controls.force_failure||
     x.controls.sensor!=y.controls.sensor||x.controls.rate_failure!=y.controls.rate_failure)return false;
  for(unsigned i=0;i<ChannelCount;++i) {
    const auto& c=x.channels[i];const auto& d=y.channels[i];
    if(c.curve_index!=d.curve_index||c.hysteresis!=d.hysteresis||!Same(c.stiffness,d.stiffness)||
       !Same(c.ordinate_scale,d.ordinate_scale)||!Same(c.abscissa_scale,d.abscissa_scale)||
       !Same(c.damping,d.damping)||!Same(c.failure_negative,d.failure_negative)||
       !Same(c.failure_positive,d.failure_positive)||!Same(c.failure_weight,d.failure_weight)||
       !Same(c.failure_exponent,d.failure_exponent))return false;
  }
  for(unsigned i=0;i<CurveCount;++i)for(unsigned j=0;j<CurvePoints;++j)
    if(!Same(x.curves[i].points[j].x,y.curves[i].points[j].x)||
       !Same(x.curves[i].points[j].y,y.curves[i].points[j].y))return false;
  return true;
}
bool Same(const ModelConnection& a,const ModelConnection& b) noexcept {
  if(a.source_id!=b.source_id||a.property!=b.property||!Same(a.skew_x,b.skew_x)||
     !Same(a.skew_y,b.skew_y)||!Same(a.coordinate_noise,b.coordinate_noise))return false;
  for(unsigned i=0;i<3;++i)if(a.node[i]!=b.node[i])return false;
  for(unsigned i=0;i<4;++i)if(a.endpoint_release[i]!=b.endpoint_release[i])return false;
  return true;
}
} // namespace tl::fea::type13::model_detail
