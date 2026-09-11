// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../Model.h"
#include "../Type45Reference.h"
#include "lib_utils/BoundedArena.h"
#include "lib_utils/SourceIdentityIndex.h"

namespace tl::fea::type45::model_detail {
using Index = util::SourceIdentityIndex<0>;
struct Layout {
  util::ArenaRegion joints;
  std::size_t arena = 0, owned = 0, startup = 0;
};
inline ModelReport Error(ModelStatus status,const char* text,std::size_t joint=SIZE_MAX,
                         std::size_t slot=SIZE_MAX) { return {status,text,joint,slot}; }
ModelReport Preflight(const NodalRigidAssemblyBinding&,ModelInput,ModelLimits,std::size_t,Layout&);
ModelReport PrepareJoint(const NodalRigidAssemblyBinding&,const JointInput&,Joint&);
} // namespace tl::fea::type45::model_detail
