#pragma once

#include "Q4ContactBounds.h"
#include "SurfaceContactLaw.h"

namespace tlfea::contact {
namespace q4_integration {
using q4_bounds::Interval;

TL_SURFACE_HD inline Q4IntegrationReport Failure(Q4IntegrationStatus status, Status cause,
                                                std::uint32_t cell=UINT32_MAX,
                                                std::uint32_t depth=0, std::uint32_t visited=0,
                                                std::uint32_t leaves=0) {
  return {status,cause,cell,depth,visited,leaves};
}
TL_SURFACE_HD inline Q4IntegrationReport FromStatus(Status status) {
  const auto kind=status == Status::kNoDynamicDofs ? Q4IntegrationStatus::NoDynamicDofs :
      status == Status::kUnsupportedInterpolation ? Q4IntegrationStatus::UnsupportedInput :
      status == Status::kNonFiniteResult ? Q4IntegrationStatus::NonFiniteArithmetic :
      Q4IntegrationStatus::InvalidInput;
  return Failure(kind,status);
}
TL_SURFACE_HD inline bool CellBounds(const Q4NormalIntegrationInput& input,
                                     const Interval parent_gap[4], Q4IntegrationCell* cell) {
  double shape[4][4]; Interval gap[4],area;
  if (!q4_bounds::CellCorners(*cell,shape) ||
      !q4_bounds::CellArea(input.projected_area,cell->depth,&area)) return false;
  bool nonpositive=true,nonnegative=true;
  double penetration=0;
  for (unsigned i=0;i<4;++i) {
    if (!q4_bounds::Restrict(parent_gap,shape[i],&gap[i])) return false;
    nonpositive=nonpositive && gap[i].upper <= 0;
    nonnegative=nonnegative && gap[i].lower >= 0;
    if (gap[i].upper > penetration) penetration=gap[i].upper;
  }
  if (nonpositive) { cell->kind=Q4IntegrationCellKind::Inactive; return true; }
  cell->kind=nonnegative ? Q4IntegrationCellKind::Active : Q4IntegrationCellKind::Mixed;
  return nonnegative ? q4_bounds::ActiveMoments(gap,shape,area,input.stiffness_per_area,cell->integrals) :
      q4_bounds::MixedBounds(penetration,shape,area,input.stiffness_per_area,cell->integrals);
}

// The heap only chooses which unresolved cell to refine. Its rounded priority
// is never interpreted as a physical error certificate.
TL_SURFACE_HD inline double Priority(const Q4IntegrationCell& cell,const Q4IntegrationLimits& limits) {
  double force=0;
  for (unsigned i=0;i<4;++i) force+=cell.integrals[i].upper-cell.integrals[i].lower;
  const double force_ratio=force/limits.force_error;
  const double energy_ratio=(cell.integrals[4].upper-cell.integrals[4].lower)/limits.energy_error;
  return force_ratio > energy_ratio ? force_ratio : energy_ratio;
}
TL_SURFACE_HD inline bool Higher(std::uint32_t a,std::uint32_t b,
                                 Q4IntegrationScratch scratch,const Q4IntegrationLimits& limits) {
  const double pa=Priority(scratch.leaves[a],limits),pb=Priority(scratch.leaves[b],limits);
  return pa > pb || (pa == pb && a < b);
}
TL_SURFACE_HD inline void Push(std::uint32_t leaf,Q4IntegrationScratch scratch,
                               const Q4IntegrationLimits& limits,std::uint32_t* size) {
  std::uint32_t i=(*size)++;
  while (i > 0) {
    const auto parent=(i-1)/2;
    if (!Higher(leaf,scratch.heap[parent],scratch,limits)) break;
    scratch.heap[i]=scratch.heap[parent]; i=parent;
  }
  scratch.heap[i]=leaf;
}
TL_SURFACE_HD inline std::uint32_t Pop(Q4IntegrationScratch scratch,const Q4IntegrationLimits& limits,
                                     std::uint32_t* size) {
  const auto result=scratch.heap[0],last=scratch.heap[--(*size)];
  if (*size == 0) return result;
  std::uint32_t i=0;
  while (2*i+1 < *size) {
    auto child=2*i+1;
    if (child+1 < *size && Higher(scratch.heap[child+1],scratch.heap[child],scratch,limits)) ++child;
    if (!Higher(scratch.heap[child],last,scratch,limits)) break;
    scratch.heap[i]=scratch.heap[child]; i=child;
  }
  scratch.heap[i]=last; return result;
}
TL_SURFACE_HD inline bool Accumulate(Interval totals[5],const Q4IntegrationCell& cell,bool remove) {
  for (unsigned i=0;i<5;++i) {
    if (!remove) {
      if (!q4_bounds::Add(totals[i],cell.integrals[i],&totals[i])) return false;
    } else {
      if (!q4_bounds::AddScalar(totals[i].lower,-cell.integrals[i].lower,false,&totals[i].lower) ||
          !q4_bounds::AddScalar(totals[i].upper,-cell.integrals[i].upper,true,&totals[i].upper)) return false;
      // Intersect an outward lower aggregate with the known nonnegative cone.
      // This changes a bound, never a quadrature estimate.
      if (totals[i].lower < 0) totals[i].lower=0;
    }
    if (!q4_bounds::Nonnegative(totals[i])) return false;
  }
  return true;
}
TL_SURFACE_HD inline bool WidthReady(const Interval totals[5],const Q4IntegrationLimits& limits) {
  Interval force;
  for (unsigned i=0;i<4;++i) if (!q4_bounds::Add(force,totals[i],&force)) return false;
  double force_width=0,energy_width=0;
  return q4_bounds::AbsoluteDifferenceUpper(force.upper,force.lower,&force_width) &&
      q4_bounds::AbsoluteDifferenceUpper(totals[4].upper,totals[4].lower,&energy_width) &&
      force_width <= limits.force_error && energy_width <= limits.energy_error;
}

// Shared prescribed Gauss-point operation. Callers supply the already checked
// weighted stiffness and accumulate samples in their declared deterministic
// order. This is the original mapping/mass/law/JT arithmetic, extracted without
// changing the scalar square integrator's sample order or error checks.
template<class Input>
TL_SURFACE_HD inline Status EstimateSample(const Input& input,
                                          const double parent_gap[4],double u,double v,
                                          double stiffness,double totals[5]) {
  const auto& parent=input.surface.parents[input.parent_index];
    const Q4Point natural{input.parent_index,u,v};
    Q4PointKinematics point;
    auto status=EvaluateQ4Point(input.surface,natural,&point);
    if (status != Status::kOk) return status;
    double penetration=0;
    for (unsigned i=0;i<4;++i) {
      const double term=point.shape[i]*parent_gap[i];
      if (!IsFinite(term) || (point.shape[i] != 0 && parent_gap[i] != 0 && term == 0))
        return Status::kNonFiniteResult;
      penetration+=term;
    }
    if (!IsFinite(penetration)) return Status::kNonFiniteResult;
    NormalJacobian jacobian;
    status=BuildQ4NormalXJacobian(input.mass,parent,u,v,input.attempt,&jacobian);
    if (status != Status::kOk) return status;
    NormalContactResponse response;
    status=EvaluateNormalContact({stiffness,0,.8},
        {-penetration,-point.velocity.x,jacobian.inverse_effective_mass},&response);
    if (status != Status::kOk) return status;
    if (penetration > 0 && (response.force <= 0 || response.elastic_energy <= 0))
      return Status::kNonFiniteResult;
    Q4NodalForces projection;
    status=ProjectQ4PointForce(input.surface,natural,{-response.force,0,0},&projection);
    if (status != Status::kOk) return status;
    for (unsigned i=0;i<4;++i) {
      const double force=-projection.forces[i].x;
      if (response.force > 0 && point.shape[i] > 0 && force == 0) return Status::kNonFiniteResult;
      totals[i]+=force;
      if (!IsFinite(totals[i]) || totals[i] < 0) return Status::kNonFiniteResult;
    }
    totals[4]+=response.elastic_energy;
    if (!IsFinite(totals[4]) || totals[4] < 0) return Status::kNonFiniteResult;
  return Status::kOk;
}

TL_SURFACE_HD inline Status EstimateCell(const Q4NormalIntegrationInput& input,
                                        const double parent_gap[4],const Q4IntegrationCell& cell,
                                        double totals[5]) {
  if (cell.kind == Q4IntegrationCellKind::Inactive) return Status::kOk;
  const double side=::ldexp(1.0,-static_cast<int>(cell.depth));
  const double weight=::ldexp(input.projected_area,-2*static_cast<int>(cell.depth)-2);
  const double stiffness=input.stiffness_per_area*weight;
  if (!IsFinite(weight) || weight <= 0 || !IsFinite(stiffness) || stiffness <= 0)
    return Status::kNonFiniteResult;
  const double abscissa=1/::sqrt(3.0);
  for (unsigned sample=0;sample<4;++sample) {
    const double u=-1+2*(cell.column+.5)*side + (sample&1 ? side*abscissa : -side*abscissa);
    const double v=-1+2*(cell.row+.5)*side + (sample&2 ? side*abscissa : -side*abscissa);
    const auto status=EstimateSample(input,parent_gap,u,v,stiffness,totals);
    if (status != Status::kOk) return status;
  }
  return Status::kOk;
}

TL_SURFACE_HD inline Q4IntegrationReport Finish(const Q4NormalIntegrationInput& input,
    const Q4IntegrationLimits& limits,Q4IntegrationScratch scratch,const Interval gap[4],
    const double nominal_gap[4],std::uint32_t count,std::uint32_t visited,Q4IntegrationResult* output) {
  Q4IntegrationResult result;
  Interval truth[5]{}; double estimate[5]{};
  for (std::uint32_t i=0;i<count;++i) {
    const auto& cell=scratch.leaves[i];
    if (!Accumulate(truth,cell,false))
      return Failure(Q4IntegrationStatus::NonFiniteArithmetic,Status::kNonFiniteResult,i,cell.depth,visited);
    const auto status=EstimateCell(input,nominal_gap,cell,estimate);
    if (status != Status::kOk) { auto failure=FromStatus(status); failure.cell=i; failure.visited=visited; return failure; }
    if (cell.depth > result.deepest_leaf) result.deepest_leaf=cell.depth;
    if (cell.kind != Q4IntegrationCellKind::Inactive) {
      Interval area;
      if (!q4_bounds::CellArea(input.projected_area,cell.depth,&area))
        return Failure(Q4IntegrationStatus::NonFiniteArithmetic,Status::kNonFiniteResult,i,cell.depth,visited);
      bool certainly_positive=false;
      if (cell.kind == Q4IntegrationCellKind::Active) {
        double shape[4][4];
        if (!q4_bounds::CellCorners(cell,shape)) return Failure(Q4IntegrationStatus::InvalidInput,Status::kInvalidArgument);
        for (unsigned corner=0;corner<4;++corner) {
          Interval local;
          if (!q4_bounds::Restrict(gap,shape[corner],&local))
            return Failure(Q4IntegrationStatus::NonFiniteArithmetic,Status::kNonFiniteResult,i,cell.depth,visited);
          certainly_positive=certainly_positive || local.lower > 0;
        }
      }
      if (!certainly_positive) area.lower=0;
      if (!q4_bounds::Add(result.active_area,area,&result.active_area))
        return Failure(Q4IntegrationStatus::NonFiniteArithmetic,Status::kNonFiniteResult,i,cell.depth,visited);
    }
  }
  const auto& parent=input.surface.parents[input.parent_index];
  Interval resultant; double force=0;
  for (unsigned i=0;i<4;++i) {
    if (!q4_bounds::Certify(estimate[i],truth[i],&result.force[i]) ||
        !q4_bounds::Add(resultant,truth[i],&resultant))
      return Failure(Q4IntegrationStatus::NonFiniteArithmetic,Status::kNonFiniteResult);
    force+=estimate[i];
    result.nodal.nodes[i]=parent.nodes[i]; result.nodal.forces[i]={-estimate[i],0,0};
  }
  if (!q4_bounds::Certify(force,resultant,&result.resultant) ||
      !q4_bounds::Certify(estimate[4],truth[4],&result.potential))
    return Failure(Q4IntegrationStatus::NonFiniteArithmetic,Status::kNonFiniteResult);
  bool accepted=result.resultant.error <= limits.force_error && result.potential.error <= limits.energy_error;
  for (unsigned i=0;i<4;++i) accepted=accepted && result.force[i].error <= limits.force_error;
  if (!accepted) return Failure(Q4IntegrationStatus::UnattainableAccuracy,Status::kOutOfRange,UINT32_MAX,0,visited);
  result.feature_id=parent.feature_id; result.parent_element_id=parent.parent_element_id;
  result.base_epoch=input.mass.base_epoch; result.attempt=input.attempt;
  result.leaf_count=count; result.visited=visited; result.valid=true;
  *output=result;
  return {Q4IntegrationStatus::Ok,Status::kOk,UINT32_MAX,result.deepest_leaf,visited,count};
}
}  // namespace q4_integration

TL_SURFACE_HD inline Q4IntegrationReport IntegrateQ4NormalContact(
    const Q4NormalIntegrationInput& input,const Q4IntegrationLimits& limits,
    Q4IntegrationScratch scratch,Q4IntegrationResult* output) {
  using namespace q4_integration;
  if (!output || !scratch.leaves || !scratch.heap || !limits.max_leaves ||
      limits.max_leaves > MaxQ4IntegrationLeaves || limits.max_leaves > scratch.leaf_capacity ||
      limits.max_leaves > scratch.heap_capacity || scratch.leaf_capacity > MaxQ4IntegrationLeaves ||
      scratch.heap_capacity > MaxQ4IntegrationLeaves || limits.max_depth > MaxQ4IntegrationDepth ||
      !limits.max_visited || limits.max_visited > MaxQ4IntegrationVisits ||
      !IsFinite(limits.force_error) || limits.force_error <= 0 ||
      !IsFinite(limits.energy_error) || limits.energy_error <= 0 ||
      !IsFinite(input.wall_x) || !IsFinite(input.projected_area) || input.projected_area <= 0 ||
      !IsFinite(input.stiffness_per_area) || input.stiffness_per_area <= 0 ||
      !IsFinite(input.max_penetration) || input.max_penetration <= 0 ||
      input.mass.node_count != input.surface.positions.node_count)
    return Failure(Q4IntegrationStatus::InvalidInput,Status::kInvalidArgument);
  Q4PointKinematics center;
  auto status=EvaluateQ4Point(input.surface,{input.parent_index,0,0},&center);
  if (status != Status::kOk) return FromStatus(status);
  const auto& parent=input.surface.parents[input.parent_index];
  NormalJacobian mass;
  status=BuildQ4NormalXJacobian(input.mass,parent,0,0,input.attempt,&mass);
  if (status != Status::kOk) return FromStatus(status);
  Interval gap[4]; double nominal_gap[4];
  for (unsigned i=0;i<4;++i) {
    const double x=input.surface.positions.at(parent.nodes[i]).x;
    nominal_gap[i]=x-input.wall_x;
    if (!q4_bounds::Difference(x,input.wall_x,&gap[i]))
      return Failure(Q4IntegrationStatus::NonFiniteArithmetic,Status::kNonFiniteResult);
    if (gap[i].upper > input.max_penetration)
      return Failure(Q4IntegrationStatus::PenetrationLimit,Status::kOutOfRange);
  }
  Q4IntegrationCell root;
  if (!CellBounds(input,gap,&root)) return Failure(Q4IntegrationStatus::NonFiniteArithmetic,Status::kNonFiniteResult,0,0,1);
  scratch.leaves[0]=root;
  std::uint32_t count=1,visited=1,heap_size=0;
  bool depth_limited=root.kind == Q4IntegrationCellKind::Mixed && limits.max_depth == 0;
  if (root.kind == Q4IntegrationCellKind::Mixed && limits.max_depth > 0) Push(0,scratch,limits,&heap_size);
  Interval totals[5]; for (unsigned i=0;i<5;++i) totals[i]=root.integrals[i];
  for (;;) {
    if (WidthReady(totals,limits)) {
      const auto finished=Finish(input,limits,scratch,gap,nominal_gap,count,visited,output);
      if (finished.status != Q4IntegrationStatus::UnattainableAccuracy) return finished;
    }
    Q4IntegrationStatus stop=Q4IntegrationStatus::Ok;
    if (!heap_size) stop=depth_limited ? Q4IntegrationStatus::DepthLimit : Q4IntegrationStatus::UnattainableAccuracy;
    else if (count+3 > limits.max_leaves) stop=Q4IntegrationStatus::LeafLimit;
    else if (visited+4 > limits.max_visited) stop=Q4IntegrationStatus::VisitLimit;
    if (stop != Q4IntegrationStatus::Ok) {
      // A width is conservative: the final rounded estimate may still satisfy
      // its actual radius budget at a resource boundary. Certify it once.
      const auto finished=Finish(input,limits,scratch,gap,nominal_gap,count,visited,output);
      if (finished.status != Q4IntegrationStatus::UnattainableAccuracy) return finished;
      return Failure(stop,Status::kOutOfRange,UINT32_MAX,
          stop == Q4IntegrationStatus::DepthLimit ? limits.max_depth : 0,visited,count);
    }
    const auto index=Pop(scratch,limits,&heap_size);
    const auto previous=scratch.leaves[index];
    Q4IntegrationCell children[4];
    for (unsigned child=0;child<4;++child) {
      auto& cell=children[child];
      cell.column=2*previous.column+(child&1); cell.row=2*previous.row+((child>>1)&1);
      cell.depth=previous.depth+1; ++visited;
      if (!CellBounds(input,gap,&cell))
        return Failure(Q4IntegrationStatus::NonFiniteArithmetic,Status::kNonFiniteResult,index,cell.depth,visited);
    }
    if (!Accumulate(totals,previous,true))
      return Failure(Q4IntegrationStatus::NonFiniteArithmetic,Status::kNonFiniteResult,index,previous.depth,visited);
    for (unsigned child=0;child<4;++child) {
      const auto next=child == 0 ? index : count++;
      scratch.leaves[next]=children[child];
      if (!Accumulate(totals,children[child],false))
        return Failure(Q4IntegrationStatus::NonFiniteArithmetic,Status::kNonFiniteResult,next,children[child].depth,visited);
      if (children[child].kind == Q4IntegrationCellKind::Mixed) {
        if (children[child].depth < limits.max_depth) Push(next,scratch,limits,&heap_size);
        else depth_limited=true;
      }
    }
  }
}

}  // namespace tlfea::contact
