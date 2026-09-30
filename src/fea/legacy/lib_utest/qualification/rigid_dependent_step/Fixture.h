// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../nodal_rigid_group/GroupStepTestSupport.h"
#include "lib_src/constraints/NodalRigidTwoMemberStep.h"

#if defined(__CUDACC__)
#define RD_STEP_HD __host__ __device__
#else
#define RD_STEP_HD
#endif
namespace rigid_dependent_test {
namespace r=tl::fea::rigid;
using rigid_step_test::Input;
using rigid_step_test::Trial;
inline Input Initial() {
  auto in=rigid_step_test::Fixture({0,1./8192,1./4096});
  in.member[0].inertia=0;in.member[1].inertia=0;
  in.member[2].mass=0;in.member[2].inertia=0;
  return in;
}
RD_STEP_HD inline r::StepStatus Evaluate(const Input& in,unsigned count,Trial& out,
    r::MemberCoefficientPolicy policy=r::MemberCoefficientPolicy::NonnegativeDependent) {
  if(count!=2&&count!=4)return r::StepStatus::InvalidInput;
  r::Vec3 x[4],f[4],c[4];
  for(unsigned k=0;k<count;++k){x[k]=in.member[k].position;f[k]=in.member[k].force;c[k]=in.member[k].couple;}
  auto body=in.body;
  if(r::AggregateWrench(body.center,x,f,c,count,body.applied)!=r::MathStatus::Success)
    return r::StepStatus::NonfiniteResult;
  Trial next;
  auto status=count==2?r::EvaluateTwoMemberPrimaryStep(body,next.primary):r::EvaluatePrimaryStep(body,next.primary);
  if(status!=r::StepStatus::Success)return status;
  for(unsigned k=0;k<count;++k) {
    status=count==2?r::EvaluateTwoMemberStep(body,next.primary,in.member[k],.001,next.member[k],policy):
      r::EvaluateMemberStep(body,next.primary,in.member[k],next.member[k],policy);
    if(status!=r::StepStatus::Success)return status;
  }
  out=next;return r::StepStatus::Success;
}
RD_STEP_HD inline void Advance(Input& in,const Trial& out,unsigned count) {
  in.body.previous_frame=out.primary.force_frame;
  in.body.center=out.primary.center;in.body.velocity=out.primary.velocity;in.body.omega=out.primary.omega;
  in.body.durations={1./4096,1./4096,1./4096};
  for(unsigned k=0;k<count;++k) {
    in.member[k].position=out.member[k].position;in.member[k].velocity=out.member[k].velocity;
    in.member[k].omega=out.member[k].omega;
  }
}
} // namespace rigid_dependent_test
#undef RD_STEP_HD
