// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "TiedPatchGeometry.h"

namespace tl::constraints::tied_shell {
// Native IRODDL1, WEIGHT1 force/moment transfer. Returns increments in ordered
// master slots; the future assembler adds repeated triangle slots to one node.
TL_TIED_PATCH_HD inline Status TransferLoad(const Patch& patch, const SecondaryLoad& load,
    MasterLoads& output) noexcept {
  using detail::math::Finite;
  if (!patch.prepared() || !Finite(load.force) || !Finite(load.couple)) {
    return Status::InvalidInput;
  }
  const auto& p = patch.values();
  const auto& r = p.secondary_offset;
  const auto& f = load.force;
  const Vec3 moment{load.couple.x + r.y*f.z - r.z*f.y,
                    load.couple.y + r.z*f.x - r.x*f.z,
                    load.couple.z + r.x*f.y - r.y*f.x};
  const auto a = detail::ApplyCofactors(p, moment);
  if (!Finite(a)) return Status::NonfiniteResult;
  MasterLoads next;
  for (unsigned i = 0; i < 4; ++i) {
    const auto& x = p.master_offset[i];
    next.force[i] = {f.x*.25 + a.y*x.z - a.z*x.y,
                     f.y*.25 + a.z*x.x - a.x*x.z,
                     f.z*.25 + a.x*x.y - a.y*x.x};
    if (!Finite(next.force[i])) return Status::NonfiniteResult;
  }
  output = next;
  return Status::Success;
}
} // namespace tl::constraints::tied_shell
