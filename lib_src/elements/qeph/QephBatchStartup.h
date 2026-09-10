#pragma once
#include "QephBatch.h"
#include "../ShellBatchFields.h"

namespace tl::fea::qeph::batch_detail {
inline bool ValidStartup(const QephBatchConfig& config,bool joined) {
  const auto& s=config.startup;
  if(!shell_batch_fields::FiniteVector(s.uniform_velocity)) return false;
  if(s.kind==BatchStartupKind::ReferenceRest)
    return s.uniform_velocity.x==0&&s.uniform_velocity.y==0&&s.uniform_velocity.z==0;
  return s.kind==BatchStartupKind::ReferenceUniformTranslation&&
         config.usage==BatchUsage::CoupledForces&&!joined;
}
// Same finite binary64 reduction on host metadata and actual device inputs.
// The host preflight admits its operation domain; it does not publish a
// measured diagnostic. The device reads each actual accepted node only after
// complete initial-motion/mass validation. Failure preserves the partial sum.
TL_QEPH_HD inline bool AddInitialTranslationKinetic(double mass,Vec3 velocity,double& kinetic) {
  const double square=shell_batch_fields::Dot(velocity,velocity);
  const double term=.5*mass*square,next=kinetic+term;
  if(!tl::math::Finite(square)||square<0||!tl::math::Finite(term)||term<0||
     !tl::math::Finite(next)||next<0) return false;
  kinetic=next; return true;
}
} // namespace tl::fea::qeph::batch_detail
