// SPDX-License-Identifier: AGPL-3.0-or-later
#include "lib_src/collision/RadiossType25TiedRemoval.h"
#include <type_traits>
int main() {
  namespace t=tlfea::contact::radioss_type25::tied_removal;
  static_assert(!std::is_convertible_v<tlfea::contact::radioss_type25::search_startup::GeometricSnapshot,
      tlfea::contact::radioss_type25::search_startup::Snapshot>);
  t::Input input;t::Snapshot output;tl::util::HostArena arena,scratch;
  if(t::Preflight(input).status!=t::Status::InvalidInput)return 1;
  return t::Build(input,{},arena,scratch,&output).status==t::Status::InvalidInput?0:2;
}
