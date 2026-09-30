// SPDX-License-Identifier: AGPL-3.0-or-later
#include "ShellPhysicalOwner.h"
#include "../solvers/NodalTrialIdentity.h"

namespace tl::fea::shell_physical_owner {
NodalReport AuthenticateInitial(const NodalCoefficientLedger& ledger,FENodalState& owner,
    const NodalStamp& expected,const ShellBatchStartup& startup,
    const NodalCinWitnessSource& source,const ProofLayout& layout) {
  if (!ledger.prepared() || !source.model || !source.model->prepared() ||
      !ledger.domain()->SharesStorage(*source.model->domain()) ||
      ledger.nodes().size()!=expected.node_count || ledger.scope().uncovered_nodes ||
      source.range_count!=source.model->rows().count ||
      source.witness_count>NodalCinLimits{}.max_witnesses ||
      (source.model->explicitly_empty() ?
        (source.range_count || source.witness_count || source.ranges || source.witnesses) :
        (!source.range_count || !source.witness_count || !source.ranges || !source.witnesses)) ||
      !expected.owner_id || expected.epoch || expected.time!=0 || expected.velocity_time!=0 ||
      expected.reactions_valid || !expected.has_rotations ||
      expected.temporal_scheme!=NodalTemporalScheme::StaggeredHalfKickStart ||
      expected.velocity_phase!=NodalVelocityPhase::Collocated ||
      !shell_startup_detail::ValidStartup(startup,true,true)) {
    return {NodalStatus::InvalidInput,"Physical contributor requires a complete fresh ledger/CIN source"};
  }
  ProofLayout checked;
  if (!ForecastProof(expected.node_count,source.range_count,layout.bytes,checked) ||
      checked.bytes!=layout.bytes ||
      checked.kinematics.offset!=layout.kinematics.offset ||
      checked.kinematics.count!=layout.kinematics.count || checked.kinematics.bytes!=layout.kinematics.bytes ||
      checked.coefficients.offset!=layout.coefficients.offset ||
      checked.coefficients.count!=layout.coefficients.count || checked.coefficients.bytes!=layout.coefficients.bytes) {
    return {NodalStatus::ResourceLimit,"Physical contributor proof layout is not its exact forecast"};
  }
  if (!trial_identity::SameStamp(expected,owner.accepted())) {
    return {NodalStatus::StaleTrial,"Physical contributor owner differs from its fresh declaration"};
  }
  auto report=owner.ValidateCinWitnessSource(source);
  if (report.status!=NodalStatus::Ok) return report;
  util::HostArena arena;
  if (!arena.Initialize(layout.bytes)) {
    return {NodalStatus::ResourceLimit,"Physical contributor initial-source proof allocation failed"};
  }
  auto* k=arena.Construct<double>(layout.kinematics);
  auto* c=arena.Construct<double>(layout.coefficients);
  if (!k || !c) return {NodalStatus::ResourceLimit,"Physical contributor initial-source layout is invalid"};
  const auto n=expected.node_count;
  const auto r=source.range_count;
  NodalStamp stamp;
  NodalSnapshotBuffer motion{k,k+3*n,n,k+6*n,k+10*n};
  report=owner.CopyAccepted(motion,&stamp);
  if (report.status!=NodalStatus::Ok) return report;
  if (!trial_identity::SameStamp(stamp,expected)) {
    return {NodalStatus::StaleTrial,"Physical contributor initial kinematics changed epoch"};
  }
  const NodalCinSnapshotBuffer raw{c,c+n,r?c+2*n:nullptr,r?c+2*n+r:nullptr,c+2*n+2*r,n,r};
  report=owner.CopyAcceptedCin(raw,&stamp);
  if (report.status!=NodalStatus::Ok) return report;
  if (!trial_identity::SameStamp(stamp,expected)) {
    return {NodalStatus::StaleTrial,"Physical contributor initial coefficients changed epoch"};
  }
  const bool constrained=startup.kind==ShellBatchStartupKind::ReferenceConstrainedUniformTranslation;
  if (constrained) {
    report=owner.ValidateInitialConstrainedTranslation(expected,k+3*n,n,startup.uniform_velocity);
    if (report.status!=NodalStatus::Ok) return report;
  }
  double kinetic=0;
  const auto nodes=ledger.domain()->nodes();
  const auto coefficients=ledger.nodes();
  for (std::size_t node=0;node<n;++node) {
    const tl::math::Vec3 position{k[3*node],k[3*node+1],k[3*node+2]};
    const tl::math::Vec3 velocity{k[3*n+3*node],k[3*n+3*node+1],k[3*n+3*node+2]};
    const tl::math::Vec3 omega{k[10*n+3*node],k[10*n+3*node+1],k[10*n+3*node+2]};
    const auto& coefficient=coefficients[node].coefficients;
    const bool motion_matches=constrained ?
        shell_startup_detail::MatchesInitialReference(position,nodes[node].position,omega,k+6*n+4*node) :
        shell_startup_detail::MatchesInitialNode(startup,position,nodes[node].position,velocity,omega,k+6*n+4*node);
    if (!motion_matches ||
        !shell_startup_detail::AddInitialTranslationKinetic(coefficient.mass,velocity,kinetic) ||
        !shell_startup_detail::SameBits(raw.mass[node],coefficient.mass) ||
        !shell_startup_detail::SameBits(raw.inertia[node],coefficient.isotropic_inertia)) {
      return {NodalStatus::InvalidInput,"Physical contributor complete initial source/coefficient identity differs",
          static_cast<std::uint32_t>(node)};
    }
  }
  return {NodalStatus::Ok,"Complete initial physical ledger and CIN owner authenticated"};
}
} // namespace tl::fea::shell_physical_owner
