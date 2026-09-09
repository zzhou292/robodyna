#pragma once

#include "T3ContactIntegrationTypes.h"
#include "SurfaceContactLaw.h"

namespace tlfea::contact::t3_integration {
using Interval=Q4IntegralInterval;
using Code=T3IntegrationStatus;
namespace bounds=q4_bounds;

struct Vertex { double shape[3]{},gap=0; Interval weight[3]{}; };
struct Piece { Vertex vertex[3]; double fraction=0; Interval area_fraction; };
struct Partition { Piece piece[2]; unsigned count=0; };
static_assert(sizeof(Partition)<2048,"Revisit the native-simplex local storage forecast");

TL_SURFACE_HD inline T3IntegrationReport Failure(Code code,Status cause=Status::kInvalidArgument,
                                                unsigned node=UINT32_MAX,unsigned sample=UINT32_MAX) {
  return {code,cause,node,sample};
}
TL_SURFACE_HD inline T3IntegrationReport FromStatus(Status status,unsigned sample=UINT32_MAX) {
  const auto code=status==Status::kNoDynamicDofs ? Code::NoDynamicDofs :
      status==Status::kNonFiniteResult ? Code::NonFiniteArithmetic :
      status==Status::kUnsupportedInterpolation ? Code::UnsupportedInput : Code::InvalidInput;
  return Failure(code,status,UINT32_MAX,sample);
}
TL_SURFACE_HD inline Vertex Corner(unsigned index,double gap) {
  Vertex v; v.shape[index]=1; v.weight[index]={1,1}; v.gap=gap; return v;
}

// The numerator and denominator are positive. A lost nonzero quotient or
// overflow rejects, as in the owning contact arithmetic. Zero/one endpoint
// cases are exact; there is no sign tolerance or snapped clipping coordinate.
TL_SURFACE_HD inline bool Quotient(double numerator,Interval denominator,Interval* output) {
  if (!IsFinite(numerator) || numerator<0 || !bounds::Nonnegative(denominator) ||
      denominator.lower<=0) return false;
  if (numerator==0) { *output={}; return true; }
  const double lo=numerator/denominator.upper,hi=numerator/denominator.lower;
  if (lo==0 || hi==0) return false;
  return bounds::Round(lo,false,&output->lower) && bounds::Round(hi,true,&output->upper);
}
TL_SURFACE_HD inline bool Crossing(unsigned positive,unsigned nonpositive,const double gap[3],
                                  Vertex* output,double* fraction,Interval* fraction_bound,
                                  double* complement,Interval* complement_bound) {
  const double a=gap[positive],b=-gap[nonpositive];
  if (!(a>0) || b<0 || !IsFinite(a) || !IsFinite(b)) return false;
  Vertex next;
  if (b==0) {
    next=Corner(nonpositive,0); *fraction=1; *fraction_bound={1,1};
    *complement=0; *complement_bound={}; *output=next; return true;
  }
  Interval denominator;
  const double nominal=a+b;
  if (!IsFinite(nominal) || !bounds::Add({a,a},{b,b},&denominator) ||
      !Quotient(a,denominator,fraction_bound) || !Quotient(b,denominator,complement_bound)) return false;
  *fraction=a/nominal; *complement=b/nominal;
  if (*fraction<=0 || *complement<=0) return false;
  next.shape[positive]=*complement; next.shape[nonpositive]=*fraction;
  next.weight[positive]=*complement_bound; next.weight[nonpositive]=*fraction_bound;
  // At the exact crossing of THIS scalar gap field the gap is exactly zero.
  *output=next; return true;
}

// Partition one fully specified scalar nodal-gap field, without subtracting a
// negative sliver from the whole triangle. All moment coefficients are positive.
TL_SURFACE_HD inline bool Clip(const double gap[3],Partition* output) {
  unsigned positive[3]{},other[3]{},np=0,nn=0,nnegative=0;
  for (unsigned i=0;i<3;++i) {
    if (!IsFinite(gap[i])) return false;
    if (gap[i]>0) positive[np++]=i; else other[nn++]=i;
    if (gap[i]<0) ++nnegative;
  }
  Partition next;
  if (!np) { *output=next; return true; } // Touching alone has zero positive-pressure area.
  if (!nnegative) {
    next.count=1; auto& p=next.piece[0]; p.fraction=1; p.area_fraction={1,1};
    for (unsigned i=0;i<3;++i) p.vertex[i]=Corner(i,gap[i]);
  } else if (np==1) {
    auto& p=next.piece[0]; double a=0,b=0,unused=0; Interval ia,ib,unused_bound;
    p.vertex[0]=Corner(positive[0],gap[positive[0]]);
    if (!Crossing(positive[0],other[0],gap,&p.vertex[1],&a,&ia,&unused,&unused_bound) ||
        !Crossing(positive[0],other[1],gap,&p.vertex[2],&b,&ib,&unused,&unused_bound) ||
        !bounds::MultiplyPositive(ia,ib,&p.area_fraction)) return false;
    p.fraction=a*b;
    if (!IsFinite(p.fraction) || p.fraction<=0) return false;
    next.count=1;
  } else {
    const unsigned i=positive[0],j=positive[1],k=other[0];
    Vertex p,q; double a=0,b=0,ca=0,cb=0; Interval ia,ib,ica,icb;
    if (!Crossing(i,k,gap,&p,&a,&ia,&ca,&ica) ||
        !Crossing(j,k,gap,&q,&b,&ib,&cb,&icb)) return false;
    next.count=2;
    next.piece[0]={{Corner(i,gap[i]),Corner(j,gap[j]),q},b,ib};
    auto& second=next.piece[1];
    second.vertex[0]=Corner(i,gap[i]); second.vertex[1]=q; second.vertex[2]=p;
    second.fraction=a*cb; // Complement computed as |g_k|/(g_j+|g_k|), never 1-beta.
    if (!IsFinite(second.fraction) || second.fraction<=0 ||
        !bounds::MultiplyPositive(ia,icb,&second.area_fraction)) return false;
  }
  *output=next; return true;
}

// Exact affine-product moments on a subtriangle of physical reference area A:
// int(N*g)=A*((sum N)*(sum g)+sum N*g)/12;
// int(g*g)=A*((sum g)^2+sum g^2)/12. The energy adds the law's factor one-half.
TL_SURFACE_HD inline bool Moments(const Partition& partition,Interval parent_area,double stiffness,
                                 Interval integral[4],Interval* active_area) {
  for (unsigned p=0;p<partition.count;++p) {
    const auto& piece=partition.piece[p]; Interval area,factor,gap_sum,gap_square_sum;
    if (!bounds::MultiplyPositive(parent_area,piece.area_fraction,&area) ||
        !bounds::Add(*active_area,area,active_area) || !bounds::DividePositive(area,12,&factor) ||
        !bounds::Scale(factor,stiffness,&factor)) return false;
    for (unsigned v=0;v<3;++v) {
      const double g=piece.vertex[v].gap; Interval square;
      if (g<0 || !bounds::Add(gap_sum,{g,g},&gap_sum) ||
          !bounds::MultiplyPositive({g,g},{g,g},&square) ||
          !bounds::Add(gap_square_sum,square,&gap_square_sum)) return false;
    }
    for (unsigned n=0;n<3;++n) {
      Interval shape_sum,diagonal,total;
      for (unsigned v=0;v<3;++v) {
        Interval term;
        if (!bounds::Add(shape_sum,piece.vertex[v].weight[n],&shape_sum) ||
            !bounds::Scale(piece.vertex[v].weight[n],piece.vertex[v].gap,&term) ||
            !bounds::Add(diagonal,term,&diagonal)) return false;
      }
      if (!bounds::MultiplyPositive(shape_sum,gap_sum,&total) || !bounds::Add(total,diagonal,&total) ||
          !bounds::MultiplyPositive(total,factor,&total) || !bounds::Add(integral[n],total,&integral[n])) return false;
    }
    Interval energy;
    if (!bounds::MultiplyPositive(gap_sum,gap_sum,&energy) ||
        !bounds::Add(energy,gap_square_sum,&energy) || !bounds::MultiplyPositive(energy,factor,&energy) ||
        !bounds::Scale(energy,.5,&energy) || !bounds::Add(integral[3],energy,&integral[3])) return false;
  }
  return true;
}

TL_SURFACE_HD inline Status Sample(const T3NormalIntegrationInput& input,const Piece& piece,
                                  const double gaps[3],unsigned sample,double stiffness,double result[4]) {
  LinearTrianglePoint point; point.triangle_index=input.triangle_index;
  for (unsigned n=0;n<3;++n) {
    point.weights[n]=0;
    for (unsigned v=0;v<3;++v) {
      const double weight=v==sample ? 2.0/3.0 : 1.0/6.0;
      const double term=weight*piece.vertex[v].shape[n];
      if (!IsFinite(term) || (piece.vertex[v].shape[n]>0 && term==0)) return Status::kNonFiniteResult;
      point.weights[n]+=term;
    }
  }
  LinearPointKinematics kinematics;
  auto status=EvaluateLinearPoint(input.surface,point,&kinematics);
  if (status!=Status::kOk) return status;
  double penetration=0;
  for (unsigned n=0;n<3;++n) {
    const double term=point.weights[n]*gaps[n];
    if (!IsFinite(term) || (point.weights[n]>0 && gaps[n]!=0 && term==0)) return Status::kNonFiniteResult;
    penetration+=term;
  }
  if (!IsFinite(penetration)) return Status::kNonFiniteResult;
  const auto& triangle=input.surface.triangles[input.triangle_index];
  NormalJacobian jacobian;
  status=BuildLinearTriangleNormalJacobian(input.mass,triangle,point.weights,nullptr,nullptr,
                                         {-1,0,0},input.attempt,&jacobian);
  if (status!=Status::kOk) return status;
  NormalContactResponse response;
  status=EvaluateNormalContact({stiffness,0,.8},
      {-penetration,-kinematics.velocity.x,jacobian.inverse_effective_mass},&response);
  if (status!=Status::kOk) return status;
  if (penetration>0 && (response.force<=0 || response.elastic_energy<=0)) return Status::kNonFiniteResult;
  TriangleNodalForces projection;
  status=ProjectLinearPointForce(input.surface,point,{-response.force,0,0},&projection);
  if (status!=Status::kOk) return status;
  for (unsigned n=0;n<3;++n) {
    const double force=-projection.forces[n].x;
    if (force<0 || (response.force>0 && point.weights[n]>0 && force==0)) return Status::kNonFiniteResult;
    result[n]+=force;
    if (!IsFinite(result[n])) return Status::kNonFiniteResult;
  }
  result[3]+=response.elastic_energy;
  return IsFinite(result[3]) && result[3]>=0 ? Status::kOk : Status::kNonFiniteResult;
}

TL_SURFACE_HD inline bool SameParent(const SurfaceTriangle& a,const SurfaceTriangle& b) {
  if (a.feature_id!=b.feature_id || a.parent_element_id!=b.parent_element_id ||
      a.parent_face_id!=b.parent_face_id || a.half_thickness!=0 || b.half_thickness!=0 ||
      a.interpolation!=SurfaceInterpolation::kLinearTriangle || b.interpolation!=a.interpolation) return false;
  for (unsigned n=0;n<3;++n) if (a.nodes[n]!=b.nodes[n]) return false;
  return true;
}
} // namespace tlfea::contact::t3_integration

namespace tlfea::contact {
TL_SURFACE_HD inline T3IntegrationReport IntegrateT3NormalContact(
    const T3NormalIntegrationInput& input,const T3IntegrationLimits& limits,T3IntegrationResult* output) {
  using namespace t3_integration;
  if (!output || !input.reference || !input.attempt ||
      !IsFinite(input.wall_x) || !IsFinite(input.stiffness_per_area) || input.stiffness_per_area<=0 ||
      !IsFinite(input.max_penetration) || input.max_penetration<=0 ||
      !IsFinite(limits.force_error) || limits.force_error<=0 ||
      !IsFinite(limits.energy_error) || limits.energy_error<=0 ||
      !input.surface.positions.valid() || !input.surface.velocities.valid() ||
      input.surface.positions.node_count!=input.surface.velocities.node_count ||
      input.surface.positions.node_count!=input.mass.node_count ||
      !input.surface.triangles || input.triangle_index>=input.surface.triangle_count ||
      input.surface.inverse_node_mass!=input.mass.inverse_mass) return Failure(Code::InvalidInput);
  const auto& parent=input.surface.triangles[input.triangle_index];
  if (!input.reference->prepared() || !SameParent(parent,input.reference->parent()))
    return Failure(Code::InvalidReference);
  LinearTrianglePoint center{input.triangle_index,{1.0/3.0,1.0/3.0,1.0/3.0}};
  NormalJacobian mass;
  auto status=BuildLinearTriangleNormalJacobian(input.mass,parent,center.weights,nullptr,nullptr,
                                               {-1,0,0},input.attempt,&mass);
  if (status!=Status::kOk) return FromStatus(status);
  LinearPointKinematics center_value;
  status=EvaluateLinearPoint(input.surface,center,&center_value);
  if (status!=Status::kOk) return FromStatus(status);
  double nominal_gap[3],lower_gap[3],upper_gap[3];
  for (unsigned n=0;n<3;++n) {
    Interval gap;
    const double coordinate=input.surface.positions.at(parent.nodes[n]).x;
    if (!bounds::Difference(coordinate,input.wall_x,&gap))
      return Failure(Code::NonFiniteArithmetic,Status::kNonFiniteResult,n);
    if (gap.upper>input.max_penetration) return Failure(Code::PenetrationLimit,Status::kOutOfRange,n);
    nominal_gap[n]=coordinate-input.wall_x; lower_gap[n]=gap.lower; upper_gap[n]=gap.upper;
  }
  // Positive shape functions make each force integral, positive-part energy
  // and positive-pressure area monotone in EVERY nodal gap. Thus the exact
  // represented-coordinate field is sandwiched by these two independently
  // clipped scalar fields, including ambiguous signs and exact touching.
  Interval lower[4]{},upper[4]{},lower_area,upper_area;
  Partition partition;
  if (!Clip(lower_gap,&partition) ||
      !Moments(partition,input.reference->area_enclosure(),input.stiffness_per_area,lower,&lower_area) ||
      !Clip(upper_gap,&partition) ||
      !Moments(partition,input.reference->area_enclosure(),input.stiffness_per_area,upper,&upper_area) ||
      !Clip(nominal_gap,&partition)) return Failure(Code::NonFiniteArithmetic,Status::kNonFiniteResult);
  T3IntegrationResult result; double estimate[4]{};
  const double area=.5*input.reference->density().value;
  if (!IsFinite(area) || area<=0) return Failure(Code::NonFiniteArithmetic,Status::kNonFiniteResult);
  for (unsigned p=0;p<partition.count;++p) {
    const double weight=area*partition.piece[p].fraction/3;
    const double stiffness=input.stiffness_per_area*weight;
    if (!IsFinite(weight) || weight<=0 || !IsFinite(stiffness) || stiffness<=0)
      return Failure(Code::NonFiniteArithmetic,Status::kNonFiniteResult);
    for (unsigned s=0;s<3;++s) {
      status=Sample(input,partition.piece[p],nominal_gap,s,stiffness,estimate);
      if (status!=Status::kOk) return FromStatus(status,3*p+s);
    }
  }
  Interval total; double force=0;
  for (unsigned n=0;n<3;++n) {
    const Interval truth{lower[n].lower,upper[n].upper};
    if (!bounds::Certify(estimate[n],truth,&result.force[n]) || !bounds::Add(total,truth,&total))
      return Failure(Code::NonFiniteArithmetic,Status::kNonFiniteResult);
    result.nodal.nodes[n]=parent.nodes[n]; result.nodal.forces[n]={-estimate[n],0,0};
    force+=estimate[n];
  }
  if (!bounds::Certify(force,total,&result.resultant) ||
      !bounds::Certify(estimate[3],{lower[3].lower,upper[3].upper},&result.potential))
    return Failure(Code::NonFiniteArithmetic,Status::kNonFiniteResult);
  bool accurate=result.resultant.error<=limits.force_error && result.potential.error<=limits.energy_error;
  for (unsigned n=0;n<3;++n) accurate=accurate && result.force[n].error<=limits.force_error;
  if (!accurate) return Failure(Code::UnattainableAccuracy,Status::kOutOfRange);
  result.active_area={lower_area.lower,upper_area.upper};
  result.feature_id=parent.feature_id; result.parent_element_id=parent.parent_element_id;
  result.parent_face_id=parent.parent_face_id; result.base_epoch=input.mass.base_epoch; result.attempt=input.attempt;
  result.subtriangle_count=partition.count; result.sample_count=3*partition.count; result.valid=true;
  *output=result; return {Code::Ok,Status::kOk};
}
} // namespace tlfea::contact
