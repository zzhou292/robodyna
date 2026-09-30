// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Binding.h"
#include "../../HistoryPhase.h"
namespace tlfea::contact::radioss_type25::selection::lifecycle::detail {
TL_MATH_HOST_DEVICE inline Status BeginRow(const Input& input,std::size_t row,RowResult& out) {
  const auto& accepted=input.accepted_rows[row];const auto& secondary=input.source.secondary[row];
  double main_coefficient=0;
  const auto& old=accepted.row;
  if(old.irtlm[0]>0&&secondary.coefficient!=0&&old.irtlm[3]==input.profile.selection.local_processor) {
    const int local=old.irtlm[2];
    if(local<1||std::size_t(local)>input.source.main_count||
       input.source.mains[local-1].global_id!=old.irtlm[0])return Status::InvalidInput;
    main_coefficient=input.source.mains[local-1].coefficient;
  }
  HistoryPhaseResult begun;
  const auto status=BeginNativeHistory(old,
      {secondary.coefficient,main_coefficient,input.profile.selection.local_processor},&begun);
  if(status!=NormalStatus::Ok)return Status::InvalidInput;
  out.history=accepted;out.history.row=begun.row;
  out.retained_count=begun.retained_candidate?1:0;
  out.initial_contact_flag=secondary.initial_contact_flag;
  // Literal OPTCD transition, after I25IRTLM but before candidate admission.
  if(out.history.row.irtlm[0]!=0)out.initial_contact_flag=0;
  return Status::Ok;
}
TL_MATH_HOST_DEVICE inline double Component(Vector value,unsigned axis) {
  return axis==0?value.x:axis==1?value.y:value.z;
}
TL_MATH_HOST_DEVICE inline double Maximum4(double a,double b,double c,double d) {
  return g::Max(g::Max(g::Max(a,b),c),d);
}
TL_MATH_HOST_DEVICE inline double Minimum4(double a,double b,double c,double d) {
  return g::Min(g::Min(g::Min(a,b),c),d);
}
TL_MATH_HOST_DEVICE inline bool OptimizedCandidate(const Input& input,std::size_t row,
    int local_main,int marker,int leave,const units_detail::Factors& units) {
  const auto& secondary=input.source.secondary[row];
  const auto& main=input.source.mains[local_main-1];
  if(secondary.coefficient==0||main.coefficient<=0||
      marker!=0||leave==-1)return false;
  const double gap=secondary.gap+main.maximum_gap;
  const double precision=input.profile.optcd_response_precision==1?
      (7.+.5)*1e-6:1e-8;
  const auto position=Position(input,secondary.node,units),velocity=Velocity(input,secondary.node,units);
  Vector points[4],velocities[4];
  for(unsigned i=0;i<4;++i) {
    points[i]=Position(input,main.nodes[i],units);velocities[i]=Velocity(input,main.nodes[i],units);
  }
  // Native local OPTCD applies z, then y, then x with inclusive endpoints.
  for(int axis=2;axis>=0;--axis) {
    const double point=Component(position,unsigned(axis)),speed=Component(velocity,unsigned(axis));
    const double v0=Component(velocities[0],unsigned(axis)),v1=Component(velocities[1],unsigned(axis));
    const double v2=Component(velocities[2],unsigned(axis)),v3=Component(velocities[3],unsigned(axis));
    const double relative=g::Max(Maximum4(v0,v1,v2,v3)-speed,speed-Minimum4(v0,v1,v2,v3));
    // Selected DRAD and DGAPLOAD are exactly zero; the source gap is GAP_M,
    // not an inferred maximum of GAP_NM or the later projected gap.
    double margin=g::Max(g::Max(gap,0.),(1.+1./100.)*relative*input.step.previous_dt);
    margin=g::Max(precision,margin);
    const double x0=Component(points[0],unsigned(axis)),x1=Component(points[1],unsigned(axis));
    const double x2=Component(points[2],unsigned(axis)),x3=Component(points[3],unsigned(axis));
    if(!(Minimum4(x0,x1,x2,x3)-margin<=point&&Maximum4(x0,x1,x2,x3)+margin>=point))return false;
  }
  return true;
}
TL_MATH_HOST_DEVICE inline void ReleaseDeletedMain(RowResult& row) {
  // MAIN_OPT_TRI releases this marker AFTER OPTCD, not before its KLEAVE gate.
  if(row.history.row.irtlm[2]<0)row.history.row.irtlm[2]=0;
}
} // namespace tlfea::contact::radioss_type25::selection::lifecycle::detail
