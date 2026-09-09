#include "NodalWallContactPoint.h"

namespace tlfea::contact {
namespace {
bool Sum(Q4CertifiedIntegral& a,const Q4CertifiedIntegral& b) {
  Q4IntegralInterval truth;
  return q4_bounds::Add({a.lower,a.upper},{b.lower,b.upper},&truth) &&
      q4_bounds::Certify(a.value+b.value,truth,&a);
}
unsigned NodeIndex(const NodalWallWeights& weights,unsigned node) {
  for (unsigned n=0;n<weights.node_count();++n) if (weights.node(n).node==node) return n;
  return weights.node_count();
}
bool AddShare(NodalWallPointResult& node,const NodalWallPointResult& share) {
  if (!node.valid) { node=share; return true; }
  if (node.node!=share.node || node.fixed!=share.fixed || node.base_epoch!=share.base_epoch ||
      node.attempt!=share.attempt || !Sum(node.force,share.force) || !Sum(node.potential,share.potential) ||
      !Sum(node.stiffness,share.stiffness)) return false;
  if (!node.fixed) {
    if (!node.row.valid || !share.row.valid || node.row.count!=1 || share.row.count!=1 ||
        !q4_bounds::AddScalar(node.row.stiffness[0],share.row.stiffness[0],true,&node.row.stiffness[0])) return false;
  }
  return true;
}
} // namespace

NodalWallReport EvaluateNodalWallContact(const NodalWallWeights& weights,VectorView positions,
    VectorView velocities,const LumpedTranslationMassView& mass,const NodalWallConfig& config,
    std::uint64_t attempt,NodalWallResult* output) {
  using Code=NodalWallStatus;
  using nodal_wall_detail::Report;
  if (!output || !attempt || !positions.valid() || !velocities.valid() ||
      !IsFinite(config.parent_force_error) || config.parent_force_error<=0 ||
      !IsFinite(config.parent_energy_error) || config.parent_energy_error<=0) return {};
  if (!weights.prepared()) return Report(Code::InvalidReference);
  if (positions.node_count!=weights.global_node_count() || velocities.node_count!=positions.node_count ||
      mass.node_count!=positions.node_count) return Report(Code::InvalidInput);
  NodalWallResult next;
  next.base_epoch=mass.base_epoch; next.attempt=attempt;
  next.parent_count=weights.parent_count(); next.node_count=weights.node_count();
  for (unsigned p=0;p<weights.parent_count();++p) {
    const auto& parent=weights.parent(p); auto& ledger=next.parents[p];
    ledger.parent_element_id=parent.parent_element_id; ledger.parent_face_id=parent.parent_face_id;
    ledger.feature_id=parent.feature_id; ledger.family=parent.family; ledger.arity=parent.arity;
    for (unsigned n=0;n<parent.arity;++n) {
      const auto node=parent.nodes[n]; NodalWallPointResult share;
      auto status=EvaluateNodalWallPoint({node,parent.share},positions.at(node),velocities.at(node),
                                         mass,config,attempt,&share);
      status.parent=p;
      if (status.status!=Code::Ok) return status;
      ledger.force[n]=share.force;
      if (!Sum(ledger.resultant,share.force) || !Sum(ledger.potential,share.potential)) {
        status.status=Code::NonFiniteArithmetic; status.cause=Status::kNonFiniteResult; return status;
      }
      const auto index=NodeIndex(weights,node);
      if (index==weights.node_count() || !AddShare(next.nodes[index],share)) {
        status.status=Code::NonFiniteArithmetic; status.cause=Status::kNonFiniteResult; return status;
      }
    }
    bool accurate=ledger.resultant.error<=config.parent_force_error && ledger.potential.error<=config.parent_energy_error;
    for (unsigned n=0;n<parent.arity;++n) accurate=accurate && ledger.force[n].error<=config.parent_force_error;
    if (!accurate) { auto failure=Report(Code::Accuracy); failure.parent=p; return failure; }
    ledger.valid=true;
  }
  // Exactly one published force/energy per unique physical node. Parent
  // ledgers are diagnostics, never another force added after this reduction.
  for (unsigned n=0;n<next.node_count;++n) {
    auto& node=next.nodes[n];
    if (!node.valid || !Sum(next.resultant,node.force) || !Sum(next.potential,node.potential))
      return Report(Code::NonFiniteArithmetic,Status::kNonFiniteResult,node.node);
    node.force_world={-node.force.value,0,0}; node.wall_reaction={node.force.value,0,0};
    node.wall_moment=geometry_detail::Cross(node.wall_point,node.wall_reaction);
    node.surface_power=Dot(node.force_world,velocities.at(node.node));
    node.local_velocity_first_timestep=0; // Min isolated-share steps is not a coupled bound.
    next.wall_reaction=Add(next.wall_reaction,node.wall_reaction);
    next.wall_moment=Add(next.wall_moment,node.wall_moment);
    next.surface_power+=node.surface_power;
    if (!IsFinite(node.wall_moment) || !IsFinite(node.surface_power) || !IsFinite(next.wall_reaction) ||
        !IsFinite(next.wall_moment) || !IsFinite(next.surface_power))
      return Report(Code::NonFiniteArithmetic,Status::kNonFiniteResult,node.node);
  }
  next.valid=true; *output=next; return Report(Code::Ok,Status::kOk);
}
} // namespace tlfea::contact
