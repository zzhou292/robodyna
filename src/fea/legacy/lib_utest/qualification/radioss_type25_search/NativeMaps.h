// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "NativeOracle.h"
#include <stdexcept>
namespace type25_search_test {
// Test boundary only: translate the caller's explicit role disposition to the
// native signed MSR roster. No production numerical helper is called.
inline int NativeMain(const s::Source& source,const s::Current& current,std::uint32_t node) {
  if(node==UINT32_MAX)return 0;
  if(source.activity_policy==s::ActivityPolicy::MonotoneRetirement){
    if(current.main_node_activity_count!=source.physical_nodes||!current.main_node_activity||
        current.main_node_activity[node]>1)throw std::invalid_argument("Native main activity descriptor");
    if(!current.main_node_activity[node])return -int(node+1);
  }
  return int(node+1);
}
}
