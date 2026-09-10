#pragma once
#include "RecurrenceAudit.h"

namespace tl::qualification::qeph::recurrence {
// The same full native map as NativeMap, with a physical common carried
// velocity (m/s) added BEFORE the normalized input perturbation. The returned
// coordinates remain relative to Model::position and reference/rest history;
// the boost and its drift are intentionally NOT subtracted from the output.
// A caller differentiating about motion must retain and check that baseline.
// Zero boost executes the original arithmetic. Failure preserves output,
// including when input and output are the same vector. This is a disposable
// prescribed probe, not a startup admission or a dynamics owner.
bool NativeMapWithUniformVelocity(const Model&,double h,const Vec3& velocity,
                                  const Eigen::VectorXd& input,
                                  Eigen::VectorXd& output,std::string& error);
} // namespace tl::qualification::qeph::recurrence
