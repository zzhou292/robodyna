// SPDX-License-Identifier: AGPL-3.0-or-later
#include "lib_src/collision/RadiossType25CurrentNormals.h"
#include <type_traits>
int main(){namespace c=tlfea::contact::radioss_type25::current_normals;
  static_assert(std::is_trivially_copyable_v<c::Input>);
  c::Forecast out{19,23};const auto report=c::Preflight({}, {},out);
  return report.status==c::Status::UnsupportedProfile&&out.scratch_bytes==19&&out.output_bytes==23?0:1;
}
