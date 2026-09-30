// SPDX-License-Identifier: AGPL-3.0-or-later
#include "lib_src/collision/RadiossType25GapSource.h"
#ifdef __FAST_MATH__
#error Precise startup arithmetic is a public usage requirement
#endif
int main() {
  namespace g=tlfea::contact::radioss_type25::source_gaps;
  g::Input in;in.node_count=1;in.profile={1,0,1,1,0,0,1.,1.e30,1.e30};
  g::Forecast f;
  return g::Preflight(in,{},f).status!=g::Status::Ok || !f.scratch_bytes || f.output_bytes;
}
