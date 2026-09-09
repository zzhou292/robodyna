#pragma once

#include "SurfaceContactTypes.h"

namespace tlfea::contact {

constexpr std::uint32_t kMaxNormalNodes = 6;
enum class TranslationMassModel : std::uint8_t {
  kUnspecified, kIsotropicLumped, kGeneralizedOrRotational
};

// One global PHYSICAL-node space, shared with FENodalStateView connectivity.
// The coordinator establishes mass/constraint ownership; epoch is provenance,
// not an owner identity. This is not FEStateBuffer's ANCF coefficient space.
// fixed[i] is explicitly 0 or 1: free nodes require positive inverse mass;
// fixed nodes require zero. An uninitialized zero mass is never inferred fixed.
// Pointers are borrowed in the executing memory space, nonaliasing and sized
// by node_count. No allocation, stream synchronization or mass lumping occurs.
struct LumpedTranslationMassView {
  const double* inverse_mass = nullptr;
  const std::uint8_t* fixed = nullptr;
  std::uint32_t node_count = 0;
  std::uint64_t base_epoch = 0;
  TranslationMassModel model = TranslationMassModel::kUnspecified;
};

struct SignedNodeWeight { std::uint32_t node = 0; double weight = 0; };
struct NormalJacobian {
  std::uint32_t count = 0;
  std::uint32_t nodes[kMaxNormalNodes]{};
  Vec3 values[kMaxNormalNodes]{};  // J_i: relative normal speed = sum J_i.v_i.
  // Padded norms ||J_i/sqrt(m_i)|| used for conservative row majorants.
  // These are not an alternative inverse-mass estimate for the force law.
  double normalized_norm[kMaxNormalNodes]{};
  double inverse_effective_mass = 0;  // sum J_i.M_i^-1.J_i, kg^-1.
  std::uint64_t base_epoch = 0, attempt = 0;
  bool valid = false;
};

namespace mass_detail {
TL_SURFACE_HD inline double Norm(Vec3 v) { return ::hypot(::hypot(v.x,v.y),v.z); }
TL_SURFACE_HD inline bool Upper(double value,double* out) {
  if (!IsFinite(value) || value < 0) return false;
  *out = value == 0 ? 0 : ::nextafter(value,HUGE_VAL);
  return IsFinite(*out);
}
TL_SURFACE_HD inline bool UpperProduct(double a,double b,double* out) {
  const double value=a*b;
  if (a>0 && b>0 && value==0) return false;  // Fail on lost positive terms.
  return Upper(value,out);
}
TL_SURFACE_HD inline bool UpperSum(double a,double b,double* out) {
  return Upper(a+b,out);
}
TL_SURFACE_HD inline bool UpperQuotient(double a,double b,double* out) {
  const double value=a/b;
  if(a>0 && value==0) return false;
  return Upper(value,out);
}
// Upper norm using rounded-up basic operations, not an assumed error bound for
// libm hypot. Admitted builds use IEEE binary64 round-to-nearest basic arithmetic
// and sqrt, with no fast-math, reassociation or flush-to-zero. Positive-term
// underflow fails. Scaling avoids otherwise unnecessary square overflow.
TL_SURFACE_HD inline bool UpperNorm(Vec3 v,double* out) {
  const double x=::fabs(v.x),y=::fabs(v.y),z=::fabs(v.z);
  const double xy=x>y?x:y,scale=xy>z?xy:z;
  if(!IsFinite(scale)) return false;
  if(scale==0) { *out=0;return true; }
  double sum=0;
  const double components[3]={x,y,z};
  for(int i=0;i<3;++i) {
    double ratio=0,square=0;
    if(!UpperQuotient(components[i],scale,&ratio) || !UpperProduct(ratio,ratio,&square) ||
       !UpperSum(sum,square,&sum)) return false;
  }
  double root=0;
  return Upper(::sqrt(sum),&root) && UpperProduct(scale,root,out);
}
TL_SURFACE_HD inline Status CheckNode(const LumpedTranslationMassView& mass,std::uint32_t node) {
  if(node>=mass.node_count) return Status::kOutOfRange;
  const double inverse=mass.inverse_mass[node];
  if(!IsFinite(inverse) || mass.fixed[node]>1 ||
     (mass.fixed[node] ? inverse!=0 : inverse<=0)) return Status::kInvalidArgument;
  return Status::kOk;
}
}  // namespace mass_detail

// A small scalar-mode stencil also admits a declared linear axial spring.
// Merge signed weights BEFORE mass normalization/squaring. Sort node IDs to
// make the output independent of endpoint order for exactly representable sums.
// Unsupported mass, overflow and positive-term underflow clear the output.
TL_SURFACE_HD inline Status BuildNormalJacobian(
    const LumpedTranslationMassView& mass,const SignedNodeWeight* weights,
    std::uint32_t count,Vec3 normal,std::uint64_t attempt,NormalJacobian* out) {
  if(!out) return Status::kInvalidArgument;
  *out={};
  if(mass.model!=TranslationMassModel::kIsotropicLumped) return Status::kUnsupportedInterpolation;
  if(!mass.inverse_mass || !mass.fixed || !mass.node_count || !weights || !count || !attempt)
    return Status::kInvalidArgument;
  if(count>kMaxNormalNodes) return Status::kOutOfRange;
  const double normal_length=mass_detail::Norm(normal);
  if(!IsFinite(normal) || !IsFinite(normal_length) || ::fabs(normal_length-1)>1e-12)
    return Status::kInvalidArgument;
  SignedNodeWeight merged[kMaxNormalNodes]{};
  std::uint32_t size=0;
  for(std::uint32_t i=0;i<count;++i) {
    if(!IsFinite(weights[i].weight)) return Status::kInvalidArgument;
    const auto status=mass_detail::CheckNode(mass,weights[i].node);
    if(status!=Status::kOk) return status;
    std::uint32_t j=0;
    while(j<size && merged[j].node<weights[i].node) ++j;
    if(j<size && merged[j].node==weights[i].node) {
      merged[j].weight+=weights[i].weight;
      if(!IsFinite(merged[j].weight)) return Status::kNonFiniteResult;
    } else {
      for(std::uint32_t k=size;k>j;--k) merged[k]=merged[k-1];
      merged[j]=weights[i];++size;
    }
  }
  NormalJacobian result;result.base_epoch=mass.base_epoch;result.attempt=attempt;
  for(std::uint32_t i=0;i<size;++i) {
    if(merged[i].weight==0) continue;
    const auto node=merged[i].node;
    const Vec3 value=Scale(normal,merged[i].weight);
    const double length=mass_detail::Norm(value),root=::sqrt(mass.inverse_mass[node]);
    const double normalized=length*root,term=normalized*normalized;
    if(!IsFinite(value) || !IsFinite(normalized) || !IsFinite(term) ||
       (!mass.fixed[node] && (length==0 || normalized==0 || term==0)))
      return Status::kNonFiniteResult;
    result.inverse_effective_mass+=term;
    if(!IsFinite(result.inverse_effective_mass)) return Status::kNonFiniteResult;
    double padded_length=0,padded_root=0,padded_norm=0;
    if(!mass_detail::UpperNorm(value,&padded_length) || !mass_detail::Upper(root,&padded_root) ||
       !mass_detail::UpperProduct(padded_length,padded_root,&padded_norm))
      return Status::kNonFiniteResult;
    const auto j=result.count++;
    result.nodes[j]=node;result.values[j]=value;result.normalized_norm[j]=padded_norm;
  }
  if(result.inverse_effective_mass==0) return Status::kNoDynamicDofs;
  result.valid=true;*out=result;return Status::kOk;
}

// Linear physical midsurface triangles only. A null B with null weights_B is
// a prescribed world endpoint (e.g. fixed mesh-wall feature); its mesh indices
// do not enter the dynamic mass budget. For two FE endpoints, both triangles
// must already use the coordinator's same global node map. Geometry discovery,
// contact area/feature ownership and normal choice remain the caller's job.
// Reject shell thickness/offset, rotational and ANCF proxies explicitly; their
// force/velocity Jacobians and mass solves require separate adapters.
TL_SURFACE_HD inline Status BuildLinearTriangleNormalJacobian(
    const LumpedTranslationMassView& mass,const SurfaceTriangle& a,const double* weights_a,
    const SurfaceTriangle* b,const double* weights_b,Vec3 normal,
    std::uint64_t attempt,NormalJacobian* out) {
  if(!out) return Status::kInvalidArgument;
  *out={};
  if(!weights_a || (b==nullptr)!=(weights_b==nullptr)) return Status::kInvalidArgument;
  SignedNodeWeight weights[kMaxNormalNodes]{};
  const SurfaceTriangle* triangles[2]={&a,b};
  const double* values[2]={weights_a,weights_b};
  const int endpoints=b?2:1;
  for(int endpoint=0;endpoint<endpoints;++endpoint) {
    const auto& triangle=*triangles[endpoint];double sum=0;
    if(triangle.interpolation!=SurfaceInterpolation::kLinearTriangle || triangle.half_thickness!=0)
      return Status::kUnsupportedInterpolation;
    for(int i=0;i<3;++i) {
      const double weight=values[endpoint][i];
      if(!IsFinite(weight) || weight<0 || weight>1) return Status::kInvalidArgument;
      for(int j=0;j<i;++j) if(triangle.nodes[i]==triangle.nodes[j]) return Status::kInvalidArgument;
      sum+=weight;weights[3*endpoint+i]={triangle.nodes[i],endpoint?-weight:weight};
    }
    if(::fabs(sum-1)>1e-12) return Status::kInvalidArgument;
  }
  return BuildNormalJacobian(mass,weights,3*endpoints,normal,attempt,out);
}

}  // namespace tlfea::contact
