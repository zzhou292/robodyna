// SPDX-License-Identifier: AGPL-3.0-or-later
#include "../../RadiossType25SearchStartup.h"
#include "../search/Ranges.h"
#include <climits>
namespace tlfea::contact::radioss_type25::search_startup {
Status ResolveMultiplier(std::uint64_t nodes,double* output) noexcept {
  if(!output || !search::detail::Span(output,1) || !nodes || nodes>INT_MAX)
    return Status::InvalidInput;
  // STARTER0 includes pinned machine.inc: BMUL0=0.20, an unsuffixed REAL4
  // literal assigned to MYREAL8. Preserve that promotion, not decimal .2.
  const double base=static_cast<double>(.20f);
  double value=base;
  if(nodes>2500000)value=base*2.;
  else if(nodes>1500000)value=base*3./2.;
  *output=value;return Status::Ok;
}
} // namespace tlfea::contact::radioss_type25::search_startup
