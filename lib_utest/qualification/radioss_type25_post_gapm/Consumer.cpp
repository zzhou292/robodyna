#include "lib_src/collision/RadiossType25CurrentNormals.h"
#include "lib_src/collision/RadiossType25FixedMainStartup.h"
int main() {
  namespace n=tlfea::contact::radioss_type25;
  n::startup::Input input;n::startup::MixedSidesSnapshot sides;n::startup::PostGapmTopology post;
  if(n::startup::PreflightMixedStarter(input,sides,post,{}).status!=n::startup::Status::UnsupportedProfile)return 1;
  n::current_normals::Topology topology;n::startup::Snapshot snapshot;
  if(n::current_normals::ValidateMixedSource(topology,snapshot).status==n::current_normals::Status::Ok)return 2;
  return n::current_normals::MixedSourceValidationBytes(1)>0?0:3;
}
