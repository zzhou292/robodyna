// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "ShellBindingIdentityIndex.h"
#include <array>

namespace tl::fea::shell_batch_detail {
using ResidentNodeIdentityIndex=shell_binding_detail::IdentityIndex<0>;
// Private startup-only index. Sorting never changes native parent traversal,
// force/history order or the first failing original parent.
class ResidentTriangleIndex {
 public:
  using Key=std::array<std::size_t,3>;
  struct Entry { Key nodes{};std::size_t occurrence=0; };
  template<class NodesAt> void Prepare(std::size_t count,NodesAt nodes_at) {
    entries_.Resize(count);count_=count;
    for(std::size_t e=0;e<count;++e)entries_[e]={Sorted(nodes_at(e)),e};
    std::sort(entries_.data(),entries_.data()+count,[](const Entry& a,const Entry& b) {
      return a.nodes<b.nodes||(a.nodes==b.nodes&&a.occurrence<b.occurrence);
    });
  }
  template<class PriorNodesAt> bool DuplicateBefore(std::size_t e,Key nodes,PriorNodesAt prior_nodes_at) const {
    const auto key=Sorted(nodes);
    if(key[0]==key[1]||key[1]==key[2]) {
      // The legacy duplicate check precedes repeated-node rejection and tests
      // subset membership even for malformed connectivity. Preserve that exact
      // precedence with one bounded scan on this already-invalid input only.
      for(std::size_t prior=0;prior<e;++prior) {
        const auto old=prior_nodes_at(prior);bool same=true;
        for(auto n:nodes)same&=std::find(old.begin(),old.end(),n)!=old.end();
        if(same)return true;
      }
      return false;
    }
    std::size_t low=0,high=count_;
    while(low<high){const auto mid=low+(high-low)/2;if(entries_[mid].nodes<key)low=mid+1;else high=mid;}
    return low<count_&&entries_[low].nodes==key&&entries_[low].occurrence<e;
  }
  static constexpr std::size_t Bytes(std::size_t count) noexcept {
    return sizeof(ResidentTriangleIndex)+Storage::ExtraBytes(count);
  }
 private:
  static Key Sorted(Key key) {std::sort(key.begin(),key.end());return key;}
  using Storage=util::BoundedStartupArray<Entry,0>;
  Storage entries_;std::size_t count_=0;
};
// Caller validates vehicle hard counts before products. The original per-node
// seen/ID arrays remain separate and retain their exact validation semantics.
inline constexpr std::size_t ResidentIndexBytes(std::size_t parents,bool triangle) noexcept {
  return ResidentNodeIdentityIndex::Bytes((triangle?3:4)*parents)+
    (triangle?ResidentTriangleIndex::Bytes(parents):0);
}
} // namespace tl::fea::shell_batch_detail
