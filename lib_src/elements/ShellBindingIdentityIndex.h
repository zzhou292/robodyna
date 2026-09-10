// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "ShellHostBindingLimits.h"
#include "../../lib_utils/BoundedStartupArray.h"
#include <algorithm>
#include <cstdint>
#include <limits>

namespace tl::fea::shell_binding_detail {
// Private startup index. Sorting never changes the input/reference/reduction
// order. Looking up the FIRST original occurrence lets the caller report the
// earliest second occurrence in its original validation order, not smallest ID.
template<std::size_t InlineCount> class IdentityIndex {
 public:
  struct Entry { std::uint64_t id=0; std::size_t occurrence=0; };
  using Storage=tl::util::BoundedStartupArray<Entry,InlineCount>;
  template<class IdentityAt> void Prepare(std::size_t count,IdentityAt identity_at) {
    entries_.Resize(count); count_=count;
    for(std::size_t i=0;i<count;++i) entries_[i]={identity_at(i),i};
    std::sort(entries_.data(),entries_.data()+count,[](const Entry& a,const Entry& b) {
      return a.id<b.id||(a.id==b.id&&a.occurrence<b.occurrence);
    });
  }
  std::size_t First(std::uint64_t id) const noexcept {
    std::size_t low=0,high=count_;
    while(low<high) {
      const auto middle=low+(high-low)/2;
      if(entries_[middle].id<id) low=middle+1; else high=middle;
    }
    return low<count_&&entries_[low].id==id?entries_[low].occurrence:
      std::numeric_limits<std::size_t>::max();
  }
  static constexpr std::size_t Bytes(std::size_t count) noexcept {
    return sizeof(IdentityIndex)+Storage::ExtraBytes(count);
  }
 private:
  Storage entries_;
  std::size_t count_=0;
};

using ParentIdentityIndex=IdentityIndex<MaxShellHostParents>;
using NodeIdentityIndex=IdentityIndex<4*MaxShellHostParents>;
using NodeSeen=tl::util::BoundedStartupArray<bool,MaxShellHostNodes>;
// Counts have already passed the binding-only hard bounds before arithmetic.
inline constexpr std::size_t ScratchBytes(std::size_t q,std::size_t t,
    std::size_t nodes,bool legacy) noexcept {
  return ParentIdentityIndex::Bytes(legacy?0:q+t)+NodeIdentityIndex::Bytes(4*q+3*t)+
    sizeof(NodeSeen)+NodeSeen::ExtraBytes(nodes);
}
} // namespace tl::fea::shell_binding_detail
