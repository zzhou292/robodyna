#pragma once

#include "lib_src/collision/SurfaceContactMass.h"

namespace tl::fea::stability {
using tlfea::contact::Status;
using tlfea::contact::NormalJacobian;
namespace arithmetic=tlfea::contact::mass_detail;
using tlfea::contact::IsFinite;
constexpr auto MaxNodes=tlfea::contact::kMaxNormalNodes;

struct RowContribution {
  std::uint32_t count=0,nodes[MaxNodes]{};
  double stiffness[MaxNodes]{},damping[MaxNodes]{}; // s^-2 and s^-1.
  std::uint64_t base_epoch=0,attempt=0;
  bool valid=false;
};

// K_e=k J^T J and C_e=c J^T J, k,c>=0. For isotropic translational mass,
// a_i=J_i/sqrt(m_i) is a THREE-VECTOR. These are block spectral-norm row
// majorants k||a_i||_2 sum_j||a_j||_2, not scalar Cartesian component rows.
// Structural axial springs and normal contacts use the same operation. General
// structural modules must provide independently qualified PSD block-row bounds
// for their complete stiffness/damping, including proper inertia when admitted.
TL_SURFACE_HD inline Status MakeRankOneContribution(
    const NormalJacobian& jacobian,double stiffness,double damping,RowContribution* out) {
  if(!out) return Status::kInvalidArgument;
  *out={};
  if(!jacobian.valid || !jacobian.count || jacobian.count>MaxNodes || !jacobian.attempt)
    return Status::kNoTrial;
  if(!IsFinite(stiffness) || stiffness<0 || !IsFinite(damping) || damping<0)
    return Status::kInvalidArgument;
  double sum=0;
  for(std::uint32_t i=0;i<jacobian.count;++i) {
    const double norm=jacobian.normalized_norm[i];
    if(!IsFinite(norm) || norm<0) return Status::kInvalidArgument;
    for(std::uint32_t j=0;j<i;++j) if(jacobian.nodes[j]==jacobian.nodes[i]) return Status::kInvalidArgument;
    if(!arithmetic::UpperSum(sum,norm,&sum)) return Status::kNonFiniteResult;
  }
  if(sum==0) return Status::kNoDynamicDofs;
  RowContribution result;result.count=jacobian.count;
  result.base_epoch=jacobian.base_epoch;result.attempt=jacobian.attempt;
  for(std::uint32_t i=0;i<result.count;++i) {
    double row=0;
    if(!arithmetic::UpperProduct(jacobian.normalized_norm[i],sum,&row) ||
       !arithmetic::UpperProduct(stiffness,row,&result.stiffness[i]) ||
       !arithmetic::UpperProduct(damping,row,&result.damping[i])) return Status::kNonFiniteResult;
    result.nodes[i]=jacobian.nodes[i];
  }
  result.valid=true;*out=result;return Status::kOk;
}

// Borrowed assembly scratch only; no physical state, solver or commit owner.
// One coordinator establishes this mass/constraint space and clears global
// FENodalForceView force/couple arrays ONCE alongside ResetRows per attempt.
// Every element/contact/load then accumulates; a batch must never reset them.
// Calls/writes are serialized (one CPU caller or CUDA thread in this slice),
// not atomic parallel scatter. Arrays have capacity entries, never alias one
// another or contributions, and remain alive until the caller's stream ends.
// Epoch/attempt reject stale contributions but do not authenticate a backend.
struct RowBounds {
  double* stiffness=nullptr;
  double* damping=nullptr;
  std::uint32_t node_count=0,capacity=0;
  std::uint64_t base_epoch=0,attempt=0;
  bool initialized=false,valid=false,sealed=false;
};

TL_SURFACE_HD inline void InvalidateRows(RowBounds* rows) { if(rows) rows->valid=false; }
namespace detail {
// Begin preserves the serial API's invalidation and validation priority. No
// array or other header field is written unless this preflight succeeds.
TL_SURFACE_HD inline Status BeginResetRows(
    RowBounds* rows, std::uint64_t epoch, std::uint64_t attempt) {
  if(!rows) return Status::kInvalidArgument;
  rows->valid=false;
  if(!rows->stiffness || !rows->damping || rows->stiffness==rows->damping || !rows->node_count || !attempt)
    return Status::kInvalidArgument;
  if(rows->capacity<rows->node_count) return Status::kOutOfRange;
  if(rows->initialized && (epoch<rows->base_epoch || (epoch==rows->base_epoch && attempt<=rows->attempt)))
    return Status::kStaleTrial;
  return Status::kOk;
}
// Call only after every live row has been cleared by the same coordinator.
TL_SURFACE_HD inline Status CompleteResetRows(
    RowBounds* rows, std::uint64_t epoch, std::uint64_t attempt) {
  rows->base_epoch=epoch;rows->attempt=attempt;rows->initialized=true;rows->sealed=false;rows->valid=true;
  return Status::kOk;
}
} // namespace detail
TL_SURFACE_HD inline Status ResetRows(RowBounds* rows,std::uint64_t epoch,std::uint64_t attempt) {
  const auto status = detail::BeginResetRows(rows, epoch, attempt);
  if (status != Status::kOk) return status;
  for(std::uint32_t i=0;i<rows->node_count;++i) { rows->stiffness[i]=0;rows->damping[i]=0; }
  return detail::CompleteResetRows(rows, epoch, attempt);
}

// All local sums validate before any row is changed. On any failure the whole
// assembly scratch is invalid: discard forces/bounds and restart a new attempt.
// Upstream local-build failures must likewise call InvalidateRows; this helper
// cannot discover missing contact batches or omitted structural contributions.
TL_SURFACE_HD inline Status AccumulateRows(RowBounds* rows,const RowContribution& contribution) {
  if(!rows) return Status::kInvalidArgument;
  const bool usable=rows->valid && rows->initialized && !rows->sealed;
  rows->valid=false;
  if(!usable || !contribution.valid) return Status::kNoTrial;
  if(rows->base_epoch!=contribution.base_epoch || rows->attempt!=contribution.attempt)
    return Status::kStaleTrial;
  if(!rows->stiffness || !rows->damping || rows->stiffness==rows->damping || !rows->node_count ||
     rows->capacity<rows->node_count || !contribution.count || contribution.count>MaxNodes)
    return Status::kInvalidArgument;
  double stiffness[MaxNodes]{},damping[MaxNodes]{};
  for(std::uint32_t i=0;i<contribution.count;++i) {
    const auto node=contribution.nodes[i];
    if(node>=rows->node_count) return Status::kOutOfRange;
    for(std::uint32_t j=0;j<i;++j) if(contribution.nodes[j]==node) return Status::kInvalidArgument;
    const double values[4]={contribution.stiffness[i],contribution.damping[i],rows->stiffness[node],rows->damping[node]};
    for(double value:values)
      if(!IsFinite(value) || value<0) return Status::kInvalidArgument;
    if(!arithmetic::UpperSum(rows->stiffness[node],contribution.stiffness[i],&stiffness[i]) ||
       !arithmetic::UpperSum(rows->damping[node],contribution.damping[i],&damping[i]))
      return Status::kNonFiniteResult;
  }
  for(std::uint32_t i=0;i<contribution.count;++i) {
    rows->stiffness[contribution.nodes[i]]=stiffness[i];rows->damping[contribution.nodes[i]]=damping[i];
  }
  rows->valid=true;return Status::kOk;
}

struct StepLimit {
  double dt=0,stiffness_bound=0,damping_bound=0;
  std::uint32_t stiffness_node=0,damping_node=0;
  std::uint64_t base_epoch=0,attempt=0;
  bool has_stiffness_or_damping=false;
};

// A copied POD limit cannot revoke itself. The coordinator must check current
// valid+sealed scratch and matching provenance before consuming a saved limit.
// This does not establish owner identity or authenticate caller-modified PODs.
TL_SURFACE_HD inline bool IsCurrentLimit(const RowBounds& rows,const StepLimit& limit) {
  return rows.valid && rows.sealed && rows.base_epoch==limit.base_epoch && rows.attempt==limit.attempt &&
         IsFinite(limit.dt) && limit.dt>0;
}

// FIXED-step v'=v-h M^-1(Kx+Cv), x'=x+h v', with constant positive diagonal
// mass on free translations and frozen symmetric PSD K,C. Let A=M^-1/2 K M^-1/2,
// B=M^-1/2 C M^-1/2, alpha>=lambda_max(A), beta>=lambda_max(B).
// Any amplification eigenvector yields z^2-(2-h*b-h^2*a)z+1-h*b=0, with
// 0<=a<=alpha,0<=b<=beta; h^2*alpha+2h*beta<4 suffices by Jury's conditions.
// Any common null mode of K and C allows physical constant-velocity drift,
// even when other modes are stiff/damped; this is not strict decay.
// Positive operations are padded upward; no fast-math/reassociation contract.
// No variable-step/staggered-damping, nonlinear/geometric stiffness, rotational
// inertia, friction, shell formulation, motion-safety or full-solver claim.
// Caller finalizes only after ALL qualified contributions have arrived. A
// successful result seals scratch; later additions invalidate scratch, not old
// copies of StepLimit. Check IsCurrentLimit before consuming a saved result. No operation
// here accepts time/state. Minimum-step failure is kOutOfRange and clears out.
namespace detail {
TL_SURFACE_HD inline Status BeginFinalizeRows(RowBounds* rows,double safety,double minimum_dt,
                                       double requested_dt,StepLimit* out,StepLimit& result) {
  if(!out) { InvalidateRows(rows);return Status::kInvalidArgument; }
  *out={};
  if(!rows) return Status::kInvalidArgument;
  const bool usable=rows->valid && rows->initialized && !rows->sealed;rows->valid=false;
  if(!usable) return Status::kNoTrial;
  if(!IsFinite(safety) || safety<=0 || safety>=1 || !IsFinite(minimum_dt) || minimum_dt<=0 ||
     !IsFinite(requested_dt) || requested_dt<minimum_dt || !rows->stiffness || !rows->damping ||
     !rows->node_count || rows->capacity<rows->node_count) return Status::kInvalidArgument;
  result={};result.base_epoch=rows->base_epoch;result.attempt=rows->attempt;result.dt=requested_dt;
  return Status::kOk;
}
TL_SURFACE_HD inline Status CompleteFinalizeRows(RowBounds* rows,double safety,
    double minimum_dt,StepLimit result,StepLimit* out) {
  const double alpha=result.stiffness_bound,beta=result.damping_bound;
  result.has_stiffness_or_damping=alpha>0 || beta>0;
  if(result.has_stiffness_or_damping) {
    double root=0;
    if(!arithmetic::Upper(::sqrt(alpha),&root)) return Status::kNonFiniteResult;
    const double omega2=2*root,scale=beta>omega2?beta:omega2;
    if(!IsFinite(scale) || scale<=0) return Status::kNonFiniteResult;
    double u=0,v=0,norm=0,denominator=0;
    if(!arithmetic::UpperQuotient(beta,scale,&u) || !arithmetic::UpperQuotient(omega2,scale,&v) ||
       !arithmetic::UpperNorm({u,v,0},&norm) || !arithmetic::UpperSum(norm,u,&denominator))
      return Status::kNonFiniteResult;
    const double numerator=::nextafter(4*safety/denominator,0.0);
    if(!(numerator>0)) return Status::kNonFiniteResult;
    // Positive overflow means the bound exceeds every finite requested step;
    // it is not a usable infinity published to a timestepper. Round finite
    // divisions downward before comparing, including at a near-unit safety.
    const double quotient=numerator/scale;
    if(quotient!=HUGE_VAL) {
      const double safe_dt=::nextafter(quotient,0.0);
      if(!(safe_dt>0)) return Status::kOutOfRange;
      if(safe_dt<result.dt) result.dt=safe_dt;
    }
    if(!IsFinite(result.dt) || result.dt<=0) return Status::kNonFiniteResult;
  }
  if(result.dt<minimum_dt) return Status::kOutOfRange; // Never clamp upward.
  rows->sealed=true;rows->valid=true;*out=result;return Status::kOk;
}
} // namespace detail
TL_SURFACE_HD inline Status FinalizeRows(RowBounds* rows,double safety,double minimum_dt,
                                       double requested_dt,StepLimit* out) {
  StepLimit result;
  const auto status=detail::BeginFinalizeRows(rows,safety,minimum_dt,requested_dt,out,result);
  if(status!=Status::kOk) return status;
  for(std::uint32_t i=0;i<rows->node_count;++i) {
    const double k=rows->stiffness[i],c=rows->damping[i];
    if(!IsFinite(k) || k<0 || !IsFinite(c) || c<0) return Status::kInvalidArgument;
    if(k>result.stiffness_bound) { result.stiffness_bound=k;result.stiffness_node=i; }
    if(c>result.damping_bound) { result.damping_bound=c;result.damping_node=i; }
  }
  return detail::CompleteFinalizeRows(rows,safety,minimum_dt,result,out);
}

}  // namespace tl::fea::stability
