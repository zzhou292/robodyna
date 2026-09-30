#pragma once

#include "Q4PlanarGeometry.h"
#include "Q4ContactBounds.h"

namespace tlfea::contact {
constexpr std::uint32_t MaxQ4PlanarNodes = 4 * MaxQ4PlanarParents;

// Immutable all-active contact Gram matrix, in ascending physical-node order.
// Includes fixed nodes for reactions; their inverse masses are zero. Entries
// enclose kappa*integral(N_i*N_j dA) using C3's exact-coordinate area enclosure.
// The rate bound applies only to free X translations of this fixed-footprint
// contact law. No legacy stability rows, shell stiffness or clock are owned.
struct Q4PlanarStiffness {
  std::uint32_t nodes[MaxQ4PlanarNodes]{}, count = 0;
  double inverse_mass[MaxQ4PlanarNodes]{};
  Q4IntegralInterval entry[MaxQ4PlanarNodes][MaxQ4PlanarNodes]{};
  double rate_bound = 0;
  bool valid = false;
};

namespace q4_stiffness_detail {
using Interval = Q4IntegralInterval;

// General signed interval product, using the same directed scalar operations
// as C2. Distinct output is staged; a nonzero product lost to zero rejects.
TL_SURFACE_HD inline bool Product(Interval a, Interval b, Interval* output) {
  if (!output || !q4_bounds::Finite(a) || !q4_bounds::Finite(b)) return false;
  Interval result;
  const double x[2]={a.lower,a.upper}, y[2]={b.lower,b.upper};
  for (unsigned i=0;i<2;++i) for (unsigned j=0;j<2;++j) {
    double lower=0,upper=0;
    if (!q4_bounds::MultiplyScalar(x[i],y[j],false,&lower) ||
        !q4_bounds::MultiplyScalar(x[i],y[j],true,&upper)) return false;
    if ((!i && !j) || lower < result.lower) result.lower=lower;
    if ((!i && !j) || upper > result.upper) result.upper=upper;
  }
  *output=result;
  return true;
}

TL_SURFACE_HD inline std::uint32_t Find(const Q4PlanarStiffness& value,std::uint32_t node) {
  for (std::uint32_t i=0;i<value.count;++i) if (value.nodes[i]==node) return i;
  return MaxQ4PlanarNodes;
}
}  // namespace q4_stiffness_detail

// Metadata must originate from Q4PlanarGeometry::Initialize. Mass is borrowed
// in the same execution memory space, with masks 6/7 checked by C1. Neither
// prepared metadata nor this POD authenticates a nodal owner; C4 binds it.
// All caller output remains unchanged on failure. Outside-only geometry has
// a valid zero Gram matrix and zero rate; no positive contact rate is invented.
TL_SURFACE_HD inline PlanarContactStatus BuildQ4PlanarStiffness(
    Q4PlanarReferenceView reference,const Q4FixedYZMassView& mass,
    double stiffness_per_area,Q4PlanarStiffness* output) {
  using PStatus=PlanarContactStatus;
  if (!reference.parents || !reference.parent_count || reference.parent_count>MaxQ4PlanarParents ||
      !reference.global_node_count) return PStatus::NotInitialized;
  if (!output || !mass.inverse_mass || !mass.translation_fixed_bits ||
      mass.node_count!=reference.global_node_count || !IsFinite(stiffness_per_area) || stiffness_per_area<=0)
    return PStatus::InvalidInput;
  Q4PlanarStiffness candidate;
  for (std::uint32_t p=0;p<reference.parent_count;++p) {
    const auto& parent=reference.parents[p];
    NormalJacobian probe;
    if (BuildQ4NormalXJacobian(mass,parent.parent,0,0,1,&probe)!=Status::kOk ||
        !q4_bounds::Nonnegative(parent.area_enclosure) || parent.area_enclosure.lower<=0)
      return PStatus::InvalidInput;
    for (unsigned n=0;n<4;++n) {
      const auto node=parent.parent.nodes[n];
      if (q4_stiffness_detail::Find(candidate,node)!=MaxQ4PlanarNodes) continue;
      if (candidate.count==MaxQ4PlanarNodes) return PStatus::InvalidInput;
      unsigned at=candidate.count++;
      while (at && candidate.nodes[at-1]>node) {
        candidate.nodes[at]=candidate.nodes[at-1]; --at;
      }
      candidate.nodes[at]=node;
    }
  }
  for (unsigned i=0;i<candidate.count;++i)
    candidate.inverse_mass[i]=mass.inverse_mass[candidate.nodes[i]];
  double shape[4][4]{};
  for (unsigned i=0;i<4;++i) shape[i][i]=1;
  for (std::uint32_t p=0;p<reference.parent_count;++p) {
    const auto& parent=reference.parents[p];
    if (!parent.covered) continue;
    for (unsigned column=0;column<4;++column) {
      Q4IntegralInterval unit[4]{},moments[5]; unit[column]={1,1};
      if (!q4_bounds::ActiveMoments(unit,shape,parent.area_enclosure,stiffness_per_area,moments))
        return PStatus::InvalidOutput;
      const auto j=q4_stiffness_detail::Find(candidate,parent.parent.nodes[column]);
      for (unsigned row=0;row<4;++row) {
        const auto i=q4_stiffness_detail::Find(candidate,parent.parent.nodes[row]);
        if (!q4_bounds::Add(candidate.entry[i][j],moments[row],&candidate.entry[i][j]))
          return PStatus::InvalidOutput;
      }
    }
  }
  double root_mass[MaxQ4PlanarNodes]{};
  for (unsigned i=0;i<candidate.count;++i) {
    if (candidate.inverse_mass[i]==0) continue;
    const double root=::sqrt(candidate.inverse_mass[i]);
    if (!IsFinite(root) || root<=0 || !q4_bounds::Round(root,true,&root_mass[i]))
      return PStatus::InvalidOutput;
  }
  // The exact mass-scaled Gram is symmetric and nonnegative. Its largest
  // eigenvalue is bounded by its maximum absolute row sum. Upward entry/root/
  // accumulation operations bound that row sum, without requiring the rounded
  // upper matrix itself to have exact floating-point symmetry.
  for (unsigned i=0;i<candidate.count;++i) {
    double row=0;
    for (unsigned j=0;j<candidate.count;++j) {
      double value=0;
      if (!q4_bounds::MultiplyScalar(candidate.entry[i][j].upper,root_mass[i],true,&value) ||
          !q4_bounds::MultiplyScalar(value,root_mass[j],true,&value) ||
          !q4_bounds::AddScalar(row,value,true,&row)) return PStatus::InvalidOutput;
    }
    if (row>candidate.rate_bound) candidate.rate_bound=row;
  }
  candidate.valid=true; *output=candidate;
  return PStatus::Ok;
}

// Enclose .5*dx^T*K*dx, including subtraction uncertainty in each supplied dx.
// Fixed-node increments are allowed for prescribed work checks; the physical
// coordinator separately enforces its zero constrained increments. This upper
// bound is conservative for every partial active set of the same contact law.
TL_SURFACE_HD inline PlanarContactStatus BoundQ4PlanarQuadratic(
    const Q4PlanarStiffness& stiffness,const Q4IntegralInterval* delta,double* upper) {
  using PStatus=PlanarContactStatus;
  if (!stiffness.valid || !stiffness.count || stiffness.count>MaxQ4PlanarNodes)
    return PStatus::NotInitialized;
  if (!delta || !upper) return PStatus::InvalidInput;
  for (unsigned i=0;i<stiffness.count;++i)
    if (!q4_bounds::Finite(delta[i])) return PStatus::InvalidInput;
  Q4IntegralInterval total;
  for (unsigned i=0;i<stiffness.count;++i) {
    for (unsigned j=0;j<stiffness.count;++j) {
      Q4IntegralInterval product,term;
      if (!q4_bounds::Nonnegative(stiffness.entry[i][j])) return PStatus::InvalidOutput;
      if (stiffness.entry[i][j].upper==0) continue;
      if (!q4_stiffness_detail::Product(delta[i],delta[j],&product) ||
          !q4_stiffness_detail::Product(product,stiffness.entry[i][j],&term) ||
          !q4_bounds::Add(total,term,&total)) return PStatus::InvalidOutput;
    }
  }
  double result=0;
  if (total.upper<0 || !q4_bounds::MultiplyScalar(total.upper,.5,true,&result))
    return PStatus::InvalidOutput;
  *upper=result;
  return PStatus::Ok;
}
}  // namespace tlfea::contact
