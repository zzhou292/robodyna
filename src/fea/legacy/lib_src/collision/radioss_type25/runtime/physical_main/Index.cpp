// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Index.h"
#include "lib_utils/BoundedArena.h"
namespace tlfea::contact::radioss_type25::runtime_detail::physical_main {
namespace {
using S=TransactionStatus;
TransactionReport Ok(){return {S::Ok,"Physical source index ready"};}
std::size_t Count(const tl::fea::ShellBatchBinding& shells) {
  return shells.qeph_count()+shells.t3_count()+shells.qbat_count();
}
std::uint64_t ShellId(const tl::fea::ShellBatchBinding& shells,std::size_t index) {
  if(index<shells.qeph_count())return shells.qeph_source_id(index);
  index-=shells.qeph_count();
  return index<shells.t3_count()?shells.t3_source_id(index):shells.qbat_source_id(index-shells.t3_count());
}
}
IndexForecast Index::Preflight(const tl::fea::ShellPhysicalBinding& physical,std::size_t cap) noexcept {
  if(!physical.prepared()||!physical.domain()||!physical.shells()||!physical.coefficients()||
      !physical.coefficients()->shells()||
      !physical.coefficients()->shells()->Matches(*physical.shells(),*physical.domain()))
    return {{S::SourceMismatch,"Complete actual shell/domain mapping is required"}};
  const auto* solid=physical.coefficients()->solids();
  if(solid&&(!solid->domain()||!solid->domain()->Matches(*physical.domain())))
    return {{S::SourceMismatch,"Solid source uses a different actual domain"}};
  const auto shell_count=Count(*physical.shells()),solid_count=solid?solid->parents().size():0;
  tl::util::BoundedArenaLayout budget(cap);tl::util::ArenaRegion ignored;
  using Entry=tl::util::SourceIdentityIndex<0>::Entry;
  if(!cap||!budget.Append<std::byte>(sizeof(Index),ignored)||
      !budget.Append<Entry>(shell_count,ignored)||!budget.Append<Entry>(solid_count,ignored)||
      (shell_count&&!budget.Append<std::byte>(64,ignored))||
      (solid_count&&!budget.Append<std::byte>(64,ignored)))
    return {{S::ResourceLimit,"Complete physical source index exceeds its host cap"}};
  return {Ok(),budget.bytes()};
}
TransactionReport Index::Initialize(const tl::fea::ShellPhysicalBinding& physical,std::size_t cap) {
  if(physical_)return {S::AlreadyInitialized,"Physical source index is immutable"};
  const auto forecast=Preflight(physical,cap);if(forecast.report.status!=S::Ok)return forecast.report;
  Index staged;
  const auto& shells=*physical.shells();const auto count=Count(shells);
  if(count)staged.shells_.Prepare(count,[&](auto i){return ShellId(shells,i);});
  for(std::size_t i=0;i<count;++i)
    if(!ShellId(shells,i)||staged.shells_.First(ShellId(shells,i))!=i)
      return {S::SourceMismatch,"Repeated or absent actual shell identity",i};
  const auto* solids=physical.coefficients()->solids();
  if(solids) {
    const auto rows=solids->parents();
    if(rows.size())staged.solids_.Prepare(rows.size(),[&](auto i){return rows[i].source_element_id;});
    for(std::size_t i=0;i<rows.size();++i)
      if(!rows[i].source_element_id||staged.solids_.First(rows[i].source_element_id)!=i)
        return {S::SourceMismatch,"Repeated or absent actual solid identity",i};
  }
  staged.physical_=&physical;*this=std::move(staged);return Ok();
}
bool Index::Shell(std::uint64_t id,Face& output,bool& triangle) const noexcept {
  if(!physical_||!id)return false;
  auto index=shells_.First(id);if(index==SIZE_MAX)return false;
  const auto& shells=*physical_->shells();const auto& map=*physical_->coefficients()->shells();
  const auto q=shells.qeph_count(),t=shells.t3_count();
  triangle=index>=q&&index<q+t;Face next;
  for(unsigned corner=0;corner<4;++corner) {
    const auto local=index<q?shells.qeph_nodes(index)[corner]:
      triangle?shells.t3_nodes(index-q)[corner<3?corner:2]:shells.qbat_nodes(index-q-t)[corner];
    const auto node=map.owner_index(local);
    if(node>=physical_->domain()->node_count()||node>UINT32_MAX)return false;
    next[corner]=static_cast<std::uint32_t>(node);
  }
  output=next;return true;
}
bool Index::Solid(std::uint64_t id,std::array<std::uint32_t,8>& output) const noexcept {
  if(!physical_||!id)return false;
  const auto index=solids_.First(id);const auto* solid=physical_->coefficients()->solids();
  if(!solid||index==SIZE_MAX)return false;
  const auto& parent=solid->parents()[index];
  const bool six=parent.family==tl::fea::SolidCoefficientFamily::Solid6z;
  if(parent.node_count!=(six?6u:8u))return false;
  // Native declared-PENTA reader raw8 expansion. The coefficient snapshot
  // retains original source slots, never INITIA's later orientation order.
  constexpr unsigned penta[]{0,1,2,0,3,4,5,3};
  std::array<std::uint32_t,8> next;
  for(unsigned slot=0;slot<8;++slot) {
    const auto node=parent.domain_node[six?penta[slot]:slot];
    if(node>=physical_->domain()->node_count()||node>UINT32_MAX)return false;
    next[slot]=static_cast<std::uint32_t>(node);
  }
  output=next;return true;
}
bool SameFace(const Face& a,const Face& b) noexcept {
  const unsigned count=a[2]==a[3]?3:4;
  if(count!=(b[2]==b[3]?3u:4u))return false;
  // Native rotations/reversals preserve a face; a Q4 diagonal/bow-tie does not.
  for(unsigned start=0;start<count;++start)for(unsigned reverse=0;reverse<2;++reverse) {
    bool same=true;
    for(unsigned k=0;k<count;++k)
      same=same&&a[k]==b[(start+(reverse?count-k:k))%count];
    if(same)return true;
  }
  return false;
}
bool Contains(const std::uint32_t* nodes,std::size_t count,const Face& face) noexcept {
  for(unsigned slot=0;slot<4;++slot) {
    bool found=false;for(std::size_t k=0;k<count;++k)found=found||face[slot]==nodes[k];
    if(!found)return false;
  }
  return true;
}
} // namespace tlfea::contact::radioss_type25::runtime_detail::physical_main
