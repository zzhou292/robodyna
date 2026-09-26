// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "lib_src/collision/RadiossType25TiedRemoval.h"
#include <vector>
namespace type25_tied_removal_test {
namespace t=tlfea::contact::radioss_type25::tied_removal;
struct NativeResult {
  std::vector<std::uint32_t> main_offsets,nodes,secondary_offsets,mains;
  std::vector<t::History> history;
  std::size_t native_extent=0;
};
// Whole native serial routines behind a private COMMON lock. Caps are256nodes,
// 128target mains/secondaries,16TYPE2 interfaces,128tied mains/rows total.
NativeResult Oracle(const t::Input&);
}
