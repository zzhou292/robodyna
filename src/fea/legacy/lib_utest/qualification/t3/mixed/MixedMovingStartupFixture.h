#pragma once
#include "MixedShellFeedbackFixture.h"

namespace mixed_moving_test {
using namespace mixed_feedback_test;
inline constexpr tl::math::Vec3 Velocity{8,-2,.5};
inline fe::ShellBatchStartup MovingStartup() {
  return {fe::ShellBatchStartupKind::ReferenceUniformTranslation,Velocity};
}
bool MovingReference(Rig&);
q::QephBatchConfig QConfig(const Rig&);
t::T3BatchConfig TConfig(const Rig&);
bool Participants(Rig&);
bool BindMoving(Rig&);
bool InitializeMoving(Rig&);
void InitialKinetic(const Rig&,const Staged&,const fe::ShellBatchDiagnostics&);
} // namespace mixed_moving_test
