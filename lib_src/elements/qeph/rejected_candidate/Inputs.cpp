// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Read.h"
#include "../../ShellBatchPlasticityStorage.h"
namespace tl::fea::qeph::rejected_detail {
namespace pd=shell_batch_plasticity_detail;
namespace {
bool Same(const pd::DeviceStorage& a,const pd::DeviceStorage& b) noexcept {
  return a.curve_x==b.curve_x&&a.curve_y==b.curve_y&&a.parameters==b.parameters&&
      a.section[0]==b.section[0]&&a.section[1]==b.section[1];
}
bool Same(const pd::MixedDeviceStorage& a,const pd::MixedDeviceStorage& b) noexcept {
  return Same(a.plastic,b.plastic)&&a.law==b.law&&a.global_law1==b.global_law1&&
      a.elastic_parameters==b.elastic_parameters&&a.elastic_section[0]==b.elastic_section[0]&&
      a.elastic_section[1]==b.elastic_section[1];
}
bool Same(const pd::FailureDeviceStorage& a,const pd::FailureDeviceStorage& b) noexcept {
  return a.policy==b.policy&&a.parameters==b.parameters&&a.tab1_parameters==b.tab1_parameters&&
      a.state[0]==b.state[0]&&a.state[1]==b.state[1];
}
bool Plastic(const pd::DeviceStorage& s,std::size_t curves,std::size_t parent,unsigned slab,
    RejectedCandidateInput& out,Reader& read) noexcept {
  sections::PointParameters p;
  if(!read.Value(p,s.parameters+parent)||!read.Value(out.accepted_plastic,s.section[slab]+parent))return false;
  if(p.curve.count>MaxShellPlasticityCurvePoints)return false;
  if(p.curve.count) {
    if(p.curve.count<2||!CurveRange(p.curve.plastic_strain,s.curve_x,curves,p.curve.count)||
       !CurveRange(p.curve.yield_stress_pa,s.curve_y,curves,p.curve.count)||
       !read.Bytes(out.curve_strain,p.curve.plastic_strain,p.curve.count*sizeof(double))||
       !read.Bytes(out.curve_stress_pa,p.curve.yield_stress_pa,p.curve.count*sizeof(double)))return false;
  } else if(p.curve.plastic_strain||p.curve.yield_stress_pa)return false;
  out.plastic_parameters=CaptureMaterialParameters(p);return true;
}
}
bool ReadMaterial(const pd::HostStorage* source,std::size_t parent,unsigned slab,
                  RejectedCandidateInput& out,Reader& read) noexcept {
  if(!source) {out.route=RejectedCandidateRoute::PlainForce;return true;}
  pd::DiagnosticDeviceSources s;
  if(!source->DiagnosticSources(s)||parent>=s.parent_count||slab>1)return false;
  if(s.plain_device) {
    pd::DeviceStorage actual;
    if(!read.Value(actual,s.plain_device)||!Same(actual,s.plain)||s.mixed_device||s.failure_device)return false;
    out.route=RejectedCandidateRoute::PlasticSection;
    return Plastic(s.plain,s.curve_points,parent,slab,out,read);
  }
  pd::MixedDeviceStorage actual;
  if(!s.mixed_device||!read.Value(actual,s.mixed_device)||!Same(actual,s.mixed))return false;
  out.has_mixed=true;
  if(!read.Value(out.law,s.mixed.law+parent))return false;
  switch(out.law) {
    case ShellSectionLaw::LayeredLaw44Nip3:
      if(!Plastic(s.mixed.plastic,s.curve_points,parent,slab,out,read))return false;break;
    case ShellSectionLaw::LayeredLaw1Nip3:
      if(!read.Value(out.elastic_parameters,s.mixed.elastic_parameters+parent)||
         !read.Value(out.accepted_elastic,s.mixed.elastic_section[slab]+parent))return false;break;
    case ShellSectionLaw::GlobalLaw1Npt0:
      if(!s.mixed.global_law1||!read.Value(out.global_law1,s.mixed.global_law1+parent))return false;break;
    case ShellSectionLaw::RigidSkin:if(!out.mapped)return false;break;
    default:return false;
  }
  out.route=out.law==ShellSectionLaw::RigidSkin?RejectedCandidateRoute::RigidSkin:RejectedCandidateRoute::MixedSection;
  if(s.failure_device) {
    pd::FailureDeviceStorage actual_failure;
    if(!read.Value(actual_failure,s.failure_device)||!Same(actual_failure,s.failure)||
       !read.Value(out.failure_policy,s.failure.policy+parent)||
       !read.Value(out.accepted_failure,s.failure.state[slab]+parent))return false;
    out.has_failure=true;
    switch(out.failure_policy) {
      case ShellFailurePolicy::None:break;
      case ShellFailurePolicy::ConstantAllPoints:
        if(!read.Value(out.constant_failure,s.failure.parameters+parent))return false;break;
      case ShellFailurePolicy::Tab1AnyPoint:
        if(!read.Value(out.tab1_failure,s.failure.tab1_parameters+parent))return false;break;
      default:return false;
    }
    if(out.route!=RejectedCandidateRoute::RigidSkin)out.route=RejectedCandidateRoute::FailureSection;
  }
  return true;
}
} // namespace tl::fea::qeph::rejected_detail
