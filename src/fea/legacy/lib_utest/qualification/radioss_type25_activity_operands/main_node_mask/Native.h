// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../NativeComparison.h"
#include <algorithm>
extern "C" void robo_msr_retirement(int rows,int nodes,const int* tags,int* msr);
namespace activity_operands_test {
inline std::vector<std::uint8_t> NativeMainMask(const GpuFixture& f,const native::Mesh& mesh,
    const std::vector<std::uint8_t>& prior) {
  const auto tags=native::Tag(mesh);std::vector<int> roles;
  std::vector<std::uint8_t> member(prior.size()),result(prior.size(),1);
  for(std::size_t i=0;i<f.source.selection.main_count;++i)
    for(auto node:f.source.selection.mains[i].nodes)member.at(node)=1;
  for(std::size_t node=0;node<member.size();++node)if(member[node])
    roles.push_back((prior[node]?1:-1)*static_cast<int>(node+1));
  if(f.controls.deletion!=n::activity_source::Deletion::Disabled)
    robo_msr_retirement(static_cast<int>(roles.size()),static_cast<int>(tags.active_nodes.size()),tags.active_nodes.data(),roles.data());
  for(int role:roles)result.at(static_cast<std::size_t>(role<0?-role:role)-1)=role>0?1:0;
  return result;
}
}
