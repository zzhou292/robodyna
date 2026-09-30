#pragma once

#include "Q4ContactIntegration.h"
#include "Q4RectangularIntegrationTypes.h"

namespace tlfea::contact {
namespace q4_rectangular {
using q4_bounds::Interval;
using q4_integration::Failure;
using q4_integration::FromStatus;

// Owning resource/limit predicate, also used by bounded host composition to
// preflight every selected parent before any contributor touches scratch.
TL_SURFACE_HD inline bool ValidResources(const Q4IntegrationLimits& limits,Q4RectangularScratch scratch) {
  return !(!scratch.leaves || !scratch.heap || !limits.max_leaves ||
      limits.max_leaves>MaxQ4IntegrationLeaves || limits.max_leaves>scratch.leaf_capacity ||
      limits.max_leaves>scratch.heap_capacity || scratch.leaf_capacity>MaxQ4IntegrationLeaves ||
      scratch.heap_capacity>MaxQ4IntegrationLeaves || limits.max_depth>MaxQ4IntegrationDepth ||
      !limits.max_visited || limits.max_visited>MaxQ4IntegrationVisits ||
      !IsFinite(limits.force_error) || limits.force_error<=0 || !IsFinite(limits.energy_error) || limits.energy_error<=0);
}

TL_SURFACE_HD inline bool Corners(const Q4RectangularCell& cell,double shape[4][4]) {
  if (cell.u_depth>MaxQ4IntegrationDepth || cell.v_depth>MaxQ4IntegrationDepth ||
      cell.bounds.column>=(1u<<cell.u_depth) || cell.bounds.row>=(1u<<cell.v_depth) ||
      cell.bounds.depth!=(cell.u_depth>cell.v_depth?cell.u_depth:cell.v_depth)) return false;
  const double su=::ldexp(1.0,-static_cast<int>(cell.u_depth));
  const double sv=::ldexp(1.0,-static_cast<int>(cell.v_depth));
  const double u[2]={-1+2*cell.bounds.column*su,-1+2*(cell.bounds.column+1)*su};
  const double v[2]={-1+2*cell.bounds.row*sv,-1+2*(cell.bounds.row+1)*sv};
  const unsigned us[4]={1,0,0,1},vs[4]={1,1,0,0};
  // Both depths <=16: global corner shape products have <=34 significant
  // bits, hence are exact dyadics under the existing no-fast-math contract.
  for (unsigned i=0;i<4;++i)
    if (EvaluateQ4Shape(u[us[i]],v[vs[i]],shape[i])!=Status::kOk) return false;
  return true;
}
TL_SURFACE_HD inline bool Area(double parent,const Q4RectangularCell& cell,Interval* output) {
  return parent>0 && cell.u_depth<=MaxQ4IntegrationDepth && cell.v_depth<=MaxQ4IntegrationDepth &&
      q4_bounds::Scale({parent,parent},::ldexp(1.0,-static_cast<int>(cell.u_depth+cell.v_depth)),output);
}
template<class Input>
TL_SURFACE_HD inline bool CellBounds(const Input& input,const Interval parent_gap[4],
                                     Q4RectangularCell* cell) {
  double shape[4][4]; Interval gap[4],area;
  if (!Corners(*cell,shape) || !Area(input.projected_area,*cell,&area)) return false;
  bool nonpositive=true,nonnegative=true; double penetration=0;
  for (unsigned i=0;i<4;++i) {
    if (!q4_bounds::Restrict(parent_gap,shape[i],&gap[i])) return false;
    nonpositive=nonpositive && gap[i].upper<=0;
    nonnegative=nonnegative && gap[i].lower>=0;
    if (gap[i].upper>penetration) penetration=gap[i].upper;
  }
  if (nonpositive) { cell->bounds.kind=Q4IntegrationCellKind::Inactive; return true; }
  cell->bounds.kind=nonnegative?Q4IntegrationCellKind::Active:Q4IntegrationCellKind::Mixed;
  return nonnegative?q4_bounds::ActiveMoments(gap,shape,area,input.stiffness_per_area,cell->bounds.integrals):
      q4_bounds::MixedBounds(penetration,shape,area,input.stiffness_per_area,cell->bounds.integrals);
}

// Conservative local gap-variation upper scores. They choose work only: they
// are NEVER used as force, area or energy certificates. Saturating an overflow
// to infinity avoids changing physical admission for an ordering calculation.
TL_SURFACE_HD inline double DifferenceScore(Interval a,Interval b) {
  double first=0,second=0;
  if (!q4_bounds::AbsoluteDifferenceUpper(a.lower,b.upper,&first) ||
      !q4_bounds::AbsoluteDifferenceUpper(a.upper,b.lower,&second)) return HUGE_VAL;
  return first>second?first:second;
}
TL_SURFACE_HD inline bool SplitAxis(const Q4RectangularCell& cell,const Interval parent_gap[4],
                                    const Q4IntegrationLimits& limits,bool* split_u) {
  if (cell.u_depth>=limits.max_depth && cell.v_depth>=limits.max_depth) return false;
  double shape[4][4]; Interval gap[4];
  if (!Corners(cell,shape)) return false;
  for (unsigned i=0;i<4;++i) if (!q4_bounds::Restrict(parent_gap,shape[i],&gap[i])) return false;
  const double u0=DifferenceScore(gap[0],gap[1]),u1=DifferenceScore(gap[3],gap[2]);
  const double v0=DifferenceScore(gap[0],gap[3]),v1=DifferenceScore(gap[1],gap[2]);
  const double u=u0>u1?u0:u1,v=v0>v1?v0:v1;
  // Deterministic U tie, including two saturated scores. A capped preferred
  // axis falls back to the other axis. No nonzero variation is rounded away.
  *split_u=cell.u_depth<limits.max_depth && (cell.v_depth>=limits.max_depth || u>=v);
  return true;
}
TL_SURFACE_HD inline bool Higher(std::uint32_t a,std::uint32_t b,Q4RectangularScratch scratch,
                                 const Q4IntegrationLimits& limits) {
  const double pa=q4_integration::Priority(scratch.leaves[a].bounds,limits);
  const double pb=q4_integration::Priority(scratch.leaves[b].bounds,limits);
  return pa>pb || (pa==pb && a<b);
}
TL_SURFACE_HD inline void Push(std::uint32_t leaf,Q4RectangularScratch scratch,
                               const Q4IntegrationLimits& limits,std::uint32_t* size) {
  std::uint32_t i=(*size)++;
  while (i>0) {
    const auto parent=(i-1)/2;
    if (!Higher(leaf,scratch.heap[parent],scratch,limits)) break;
    scratch.heap[i]=scratch.heap[parent]; i=parent;
  }
  scratch.heap[i]=leaf;
}
TL_SURFACE_HD inline std::uint32_t Pop(Q4RectangularScratch scratch,const Q4IntegrationLimits& limits,
                                     std::uint32_t* size) {
  const auto result=scratch.heap[0],last=scratch.heap[--(*size)];
  if (!*size) return result;
  std::uint32_t i=0;
  while (2*i+1<*size) {
    auto child=2*i+1;
    if (child+1<*size && Higher(scratch.heap[child+1],scratch.heap[child],scratch,limits)) ++child;
    if (!Higher(scratch.heap[child],last,scratch,limits)) break;
    scratch.heap[i]=scratch.heap[child]; i=child;
  }
  scratch.heap[i]=last; return result;
}
template<class Input>
TL_SURFACE_HD inline Status EstimateCell(const Input& input,const double gap[4],
                                        const Q4RectangularCell& cell,double totals[5]) {
  if (cell.bounds.kind==Q4IntegrationCellKind::Inactive) return Status::kOk;
  const double su=::ldexp(1.0,-static_cast<int>(cell.u_depth));
  const double sv=::ldexp(1.0,-static_cast<int>(cell.v_depth));
  const double weight=::ldexp(input.projected_area,-static_cast<int>(cell.u_depth+cell.v_depth)-2);
  const double stiffness=input.stiffness_per_area*weight;
  if (!IsFinite(weight) || weight<=0 || !IsFinite(stiffness) || stiffness<=0) return Status::kNonFiniteResult;
  const double abscissa=1/::sqrt(3.0);
  for (unsigned sample=0;sample<4;++sample) {
    const double u=-1+2*(cell.bounds.column+.5)*su+(sample&1?su*abscissa:-su*abscissa);
    const double v=-1+2*(cell.bounds.row+.5)*sv+(sample&2?sv*abscissa:-sv*abscissa);
    const auto status=q4_integration::EstimateSample(input,gap,u,v,stiffness,totals);
    if (status!=Status::kOk) return status;
  }
  return Status::kOk;
}

template<class Input>
TL_SURFACE_HD inline Q4IntegrationReport Finish(const Input& input,
    const Q4IntegrationLimits& limits,Q4RectangularScratch scratch,const Interval gap[4],
    const double nominal_gap[4],std::uint32_t count,std::uint32_t visited,Q4RectangularResult* output) {
  Q4RectangularResult candidate; auto& result=candidate.integration;
  Interval truth[5]{}; double estimate[5]{};
  for (std::uint32_t i=0;i<count;++i) {
    const auto& cell=scratch.leaves[i]; const auto& bounds=cell.bounds;
    if (!q4_integration::Accumulate(truth,bounds,false))
      return Failure(Q4IntegrationStatus::NonFiniteArithmetic,Status::kNonFiniteResult,i,bounds.depth,visited);
    const auto status=EstimateCell(input,nominal_gap,cell,estimate);
    if (status!=Status::kOk) { auto failure=FromStatus(status); failure.cell=i; failure.visited=visited; return failure; }
    if (cell.u_depth>candidate.deepest_u) candidate.deepest_u=cell.u_depth;
    if (cell.v_depth>candidate.deepest_v) candidate.deepest_v=cell.v_depth;
    if (bounds.depth>result.deepest_leaf) result.deepest_leaf=bounds.depth;
    if (bounds.kind!=Q4IntegrationCellKind::Inactive) {
      Interval area;
      if (!Area(input.projected_area,cell,&area))
        return Failure(Q4IntegrationStatus::NonFiniteArithmetic,Status::kNonFiniteResult,i,bounds.depth,visited);
      bool certainly_positive=false;
      if (bounds.kind==Q4IntegrationCellKind::Active) {
        double shape[4][4];
        if (!Corners(cell,shape)) return Failure(Q4IntegrationStatus::InvalidInput,Status::kInvalidArgument);
        for (unsigned corner=0;corner<4;++corner) {
          Interval local;
          if (!q4_bounds::Restrict(gap,shape[corner],&local))
            return Failure(Q4IntegrationStatus::NonFiniteArithmetic,Status::kNonFiniteResult,i,bounds.depth,visited);
          certainly_positive=certainly_positive || local.lower>0;
        }
      }
      if (!certainly_positive) area.lower=0;
      if (!q4_bounds::Add(result.active_area,area,&result.active_area))
        return Failure(Q4IntegrationStatus::NonFiniteArithmetic,Status::kNonFiniteResult,i,bounds.depth,visited);
    }
  }
  const auto& parent=input.surface.parents[input.parent_index];
  Interval resultant; double force=0;
  for (unsigned i=0;i<4;++i) {
    if (!q4_bounds::Certify(estimate[i],truth[i],&result.force[i]) || !q4_bounds::Add(resultant,truth[i],&resultant))
      return Failure(Q4IntegrationStatus::NonFiniteArithmetic,Status::kNonFiniteResult);
    force+=estimate[i]; result.nodal.nodes[i]=parent.nodes[i]; result.nodal.forces[i]={-estimate[i],0,0};
  }
  if (!q4_bounds::Certify(force,resultant,&result.resultant) || !q4_bounds::Certify(estimate[4],truth[4],&result.potential))
    return Failure(Q4IntegrationStatus::NonFiniteArithmetic,Status::kNonFiniteResult);
  bool accepted=result.resultant.error<=limits.force_error && result.potential.error<=limits.energy_error;
  for (unsigned i=0;i<4;++i) accepted=accepted && result.force[i].error<=limits.force_error;
  if (!accepted) return Failure(Q4IntegrationStatus::UnattainableAccuracy,Status::kOutOfRange,UINT32_MAX,0,visited);
  result.feature_id=parent.feature_id; result.parent_element_id=parent.parent_element_id;
  result.base_epoch=input.mass.base_epoch; result.attempt=input.attempt;
  result.leaf_count=count; result.visited=visited; result.valid=true;
  *output=candidate;
  return {Q4IntegrationStatus::Ok,Status::kOk,UINT32_MAX,result.deepest_leaf,visited,count};
}
// Shared implementation only. Public entry points keep their distinct mass
// contracts; overload resolution selects the actual Q4 mass/Jacobian adapter.
// No sample order, refinement priority, arithmetic or budget is changed here.
template<class Input>
TL_SURFACE_HD inline Q4IntegrationReport Integrate(
    const Input& input,const Q4IntegrationLimits& limits,
    Q4RectangularScratch scratch,Q4RectangularResult* output) {
  if (!output || !ValidResources(limits,scratch) ||
      !IsFinite(input.wall_x) || !IsFinite(input.projected_area) || input.projected_area<=0 ||
      !IsFinite(input.stiffness_per_area) || input.stiffness_per_area<=0 ||
      !IsFinite(input.max_penetration) || input.max_penetration<=0 || input.mass.node_count!=input.surface.positions.node_count)
    return Failure(Q4IntegrationStatus::InvalidInput,Status::kInvalidArgument);
  Q4PointKinematics center;
  auto status=EvaluateQ4Point(input.surface,{input.parent_index,0,0},&center);
  if (status!=Status::kOk) return FromStatus(status);
  const auto& parent=input.surface.parents[input.parent_index]; NormalJacobian mass;
  status=BuildQ4NormalXJacobian(input.mass,parent,0,0,input.attempt,&mass);
  if (status!=Status::kOk) return FromStatus(status);
  Interval gap[4]; double nominal_gap[4];
  for (unsigned i=0;i<4;++i) {
    const double x=input.surface.positions.at(parent.nodes[i]).x;
    nominal_gap[i]=x-input.wall_x;
    if (!q4_bounds::Difference(x,input.wall_x,&gap[i]))
      return Failure(Q4IntegrationStatus::NonFiniteArithmetic,Status::kNonFiniteResult);
    if (gap[i].upper>input.max_penetration) return Failure(Q4IntegrationStatus::PenetrationLimit,Status::kOutOfRange);
  }
  Q4RectangularCell root;
  if (!CellBounds(input,gap,&root)) return Failure(Q4IntegrationStatus::NonFiniteArithmetic,Status::kNonFiniteResult,0,0,1);
  scratch.leaves[0]=root;
  std::uint32_t count=1,visited=1,heap_size=0;
  bool depth_limited=root.bounds.kind==Q4IntegrationCellKind::Mixed && limits.max_depth==0;
  if (root.bounds.kind==Q4IntegrationCellKind::Mixed && limits.max_depth>0) Push(0,scratch,limits,&heap_size);
  Interval totals[5]; for (unsigned i=0;i<5;++i) totals[i]=root.bounds.integrals[i];
  for (;;) {
    if (q4_integration::WidthReady(totals,limits)) {
      const auto finished=Finish(input,limits,scratch,gap,nominal_gap,count,visited,output);
      if (finished.status!=Q4IntegrationStatus::UnattainableAccuracy) return finished;
    }
    Q4IntegrationStatus stop=Q4IntegrationStatus::Ok;
    if (!heap_size) stop=depth_limited?Q4IntegrationStatus::DepthLimit:Q4IntegrationStatus::UnattainableAccuracy;
    else if (count+1>limits.max_leaves) stop=Q4IntegrationStatus::LeafLimit;
    else if (visited+2>limits.max_visited) stop=Q4IntegrationStatus::VisitLimit;
    if (stop!=Q4IntegrationStatus::Ok) {
      const auto finished=Finish(input,limits,scratch,gap,nominal_gap,count,visited,output);
      if (finished.status!=Q4IntegrationStatus::UnattainableAccuracy) return finished;
      return Failure(stop,Status::kOutOfRange,UINT32_MAX,stop==Q4IntegrationStatus::DepthLimit?limits.max_depth:0,visited,count);
    }
    const auto index=Pop(scratch,limits,&heap_size); const auto previous=scratch.leaves[index];
    bool split_u=false;
    if (!SplitAxis(previous,gap,limits,&split_u))
      return Failure(Q4IntegrationStatus::NonFiniteArithmetic,Status::kNonFiniteResult,index,previous.bounds.depth,visited);
    Q4RectangularCell children[2];
    for (unsigned child=0;child<2;++child) {
      auto& cell=children[child];
      cell.u_depth=previous.u_depth+(split_u?1:0); cell.v_depth=previous.v_depth+(split_u?0:1);
      cell.bounds.depth=cell.u_depth>cell.v_depth?cell.u_depth:cell.v_depth;
      cell.bounds.column=split_u?2*previous.bounds.column+child:previous.bounds.column;
      cell.bounds.row=split_u?previous.bounds.row:2*previous.bounds.row+child;
      ++visited;
      if (!CellBounds(input,gap,&cell))
        return Failure(Q4IntegrationStatus::NonFiniteArithmetic,Status::kNonFiniteResult,index,cell.bounds.depth,visited);
    }
    if (!q4_integration::Accumulate(totals,previous.bounds,true))
      return Failure(Q4IntegrationStatus::NonFiniteArithmetic,Status::kNonFiniteResult,index,previous.bounds.depth,visited);
    for (unsigned child=0;child<2;++child) {
      const auto next=child==0?index:count++;
      scratch.leaves[next]=children[child]; const auto& cell=children[child];
      if (!q4_integration::Accumulate(totals,cell.bounds,false))
        return Failure(Q4IntegrationStatus::NonFiniteArithmetic,Status::kNonFiniteResult,next,cell.bounds.depth,visited);
      if (cell.bounds.kind==Q4IntegrationCellKind::Mixed) {
        if (cell.u_depth<limits.max_depth || cell.v_depth<limits.max_depth) Push(next,scratch,limits,&heap_size);
        else depth_limited=true;
      }
    }
  }
}
} // namespace q4_rectangular

// Legacy constrained C2 entry. The complete finite rectangular footprint and
// supplied area remain C3 prerequisites. Scalar C2 remains a separate backend.
TL_SURFACE_HD inline Q4IntegrationReport IntegrateQ4NormalContactRectangular(
    const Q4NormalIntegrationInput& input,const Q4IntegrationLimits& limits,
    Q4RectangularScratch scratch,Q4RectangularResult* output) {
  return q4_rectangular::Integrate(input,limits,scratch,output);
}

// Prescribed free-XYZ/fully-fixed physical nodes with genuine lumped mass.
// This raw operation owns no finite-wall or swept-geometry admission. Use the
// host prescribed adapter for that contract. Exact same integral/core/budgets.
TL_SURFACE_HD inline Q4IntegrationReport IntegrateQ4NormalContactRectangular(
    const Q4PrescribedNormalIntegrationInput& input,const Q4IntegrationLimits& limits,
    Q4RectangularScratch scratch,Q4RectangularResult* output) {
  return q4_rectangular::Integrate(input,limits,scratch,output);
}
} // namespace tlfea::contact
