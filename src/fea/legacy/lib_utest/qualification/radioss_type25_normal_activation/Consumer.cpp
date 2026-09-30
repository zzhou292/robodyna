// SPDX-License-Identifier: AGPL-3.0-or-later
#include "lib_src/collision/RadiossType25NormalActivation.h"
int main() {
  namespace a=tlfea::contact::radioss_type25::normal_activation;
  // Public production header and rejection path have no oracle dependency.
  a::Input in;std::uint32_t main=17,node=29;
  const auto status=a::EvaluateNativeNormalActivation(in,{}, {&main,1,&node,1});
  return status==a::Status::UnsupportedProfile&&main==17&&node==29?0:1;
}
