// SPDX-License-Identifier: AGPL-3.0-or-later
#include "ShellPhysicalOwner.h"

namespace tl::fea::shell_physical_owner {
bool ForecastProof(std::size_t nodes,std::size_t attachments,std::size_t cap,
    ProofLayout& output) noexcept {
  if (!nodes || nodes>MaxActiveNodalStateNodes || !attachments ||
      attachments>NodalCinLimits{}.max_attachments) return false;
  ProofLayout next;
  util::BoundedArenaLayout layout(cap);
  if (!layout.Append<double>(13*nodes,next.kinematics) ||
      !layout.Append<double>(2*nodes+2*attachments+1,next.coefficients)) return false;
  next.bytes=layout.bytes();
  output=next;
  return true;
}
} // namespace tl::fea::shell_physical_owner
