// SPDX-License-Identifier: AGPL-3.0-or-later
#include "lib_src/collision/RadiossType25NodalSeed.h"
#include "lib_src/collision/RadiossType25ShellSource.h"
#include "lib_utils/BoundedArena.h"
#include <array>
int main() {
  namespace n=tlfea::contact::radioss_type25;
  n::NativeVolumeOccurrence volume{0,2,6};
  n::NativeStiffnessOccurrence stiffness{1,7};
  n::source_nodal::Input input{2,&volume,1,&stiffness,1};
  n::source_nodal::Forecast forecast;
  if(n::source_nodal::Preflight(input,{},forecast).status!=n::source_nodal::Status::Ok)return 1;
  tl::util::HostArena scratch;
  if(!scratch.Initialize(forecast.scratch_bytes))return 2;
  std::array<n::NativeNodalSeed,2> output;
  if(n::source_nodal::Accumulate(input,{},scratch.data(),scratch.bytes(),{output.data(),output.size()}).status!=n::source_nodal::Status::Ok)return 3;
  return output[0].volume==2&&output[0].bulk_volume==6&&output[1].existing_stiffness==7?0:4;
}
