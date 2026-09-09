#pragma once
#include "NodalWallContact.h"

namespace tlfea::contact::nodal_wall_reduction {
// Extracted from the qualified owning host reduction, preserving sorted-share
// addition order. Caller stages the complete result; these small helpers may
// modify their scratch argument on failure. No force/energy is counted twice.
TL_SURFACE_HD inline bool Sum(Q4CertifiedIntegral& a,const Q4CertifiedIntegral& b) {
  Q4IntegralInterval truth;
  return q4_bounds::Add({a.lower,a.upper},{b.lower,b.upper},&truth) &&
      q4_bounds::Certify(a.value+b.value,truth,&a);
}
TL_SURFACE_HD inline bool AddShare(NodalWallPointResult& node,const NodalWallPointResult& share) {
  if (!node.valid) { node=share; return true; }
  if (node.node!=share.node || node.fixed!=share.fixed || node.base_epoch!=share.base_epoch ||
      node.attempt!=share.attempt || !Sum(node.force,share.force) || !Sum(node.potential,share.potential) ||
      !Sum(node.stiffness,share.stiffness)) return false;
  if (!node.fixed && (!node.row.valid || !share.row.valid || node.row.count!=1 || share.row.count!=1 ||
      !q4_bounds::AddScalar(node.row.stiffness[0],share.row.stiffness[0],true,&node.row.stiffness[0]))) return false;
  return true;
}
} // namespace tlfea::contact::nodal_wall_reduction
