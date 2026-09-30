// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "NodalRigidPartTopology.h"
#include "../../lib_utils/SourceIdentityIndex.h"
#include <algorithm>
#include <limits>

namespace tl::fea::rigid {
struct NodalRigidPartTopology::Impl {
  std::uint64_t source_instance=0;
  std::size_t nparts=0,nextras=0,nmerges=0,nroots=0,nmembers=0,nother=0;
  std::size_t owned_bytes=0,startup_bytes=0;
  std::unique_ptr<PartTopologyPart[]> parts;
  std::unique_ptr<PartTopologyExtra[]> extras;
  std::unique_ptr<PartTopologyMerge[]> merges;
  std::unique_ptr<PartTopologyRoot[]> roots;
  std::unique_ptr<SourceNodeId[]> original,root_members,expected,other;
};
namespace topology_detail {
using Status=PartTopologyStatus;
using Report=PartTopologyReport;
using Index=tl::util::SourceIdentityIndex<1>;
inline Report Fail(Status s,const char* m,std::size_t row=SIZE_MAX,std::size_t member=SIZE_MAX) {
  return {s,m,row,member};
}
inline bool AddBytes(std::size_t n,std::size_t width,std::size_t& total) {
  if(n>(SIZE_MAX-total)/width)return false;
  total+=n*width;return true;
}
template<class T> bool Span(const T* p,std::size_t count) {
  if(!count)return p==nullptr;
  const auto address=reinterpret_cast<std::uintptr_t>(p);
  return p&&count<=SIZE_MAX/sizeof(T)&&address<=UINTPTR_MAX-count*sizeof(T);
}
struct Counts {std::size_t members=0,owned=0,startup=0;};
Report Preflight(const PartTopologyInput&,std::size_t impl_bytes,Counts&) noexcept;
Report CheckMembers(const PartTopologyInput&,const SourceNodeId* original,
                    const Index& members,Index& expected,Index& other);
// Templates keep the private implementation type inside the model's ownership;
// helpers only operate on its bounded, unpublished staging storage.
template<class Storage> Report CopyParts(const PartTopologyInput& in,Storage& s,Index& parts,Index& sets) {
  std::size_t offset=0;
  for(std::size_t i=0;i<in.part_count;++i) {
    const auto& p=in.parts[i];
    if(!p.source_part_id)return Fail(Status::InvalidInput,"Missing original rigid PART ID",i);
    s.parts[i]={p.source_part_id,offset,p.node_count,SIZE_MAX,SIZE_MAX};
    std::copy_n(p.nodes,p.node_count,s.original.get()+offset);offset+=p.node_count;
  }
  parts.Prepare(in.part_count,[&](std::size_t i){return s.parts[i].source_part_id;});
  for(std::size_t i=0;i<in.part_count;++i)
    if(parts.First(s.parts[i].source_part_id)!=i)return Fail(Status::DuplicateIdentity,"Repeated rigid PART ID",i);
  sets.Prepare(in.extra_count,[&](std::size_t i){return in.extras[i].source_node_set_id;});
  for(std::size_t i=0;i<in.extra_count;++i) {
    const auto& x=in.extras[i];const auto part=parts.First(x.source_part_id);
    if(!x.source_node_set_id||part==SIZE_MAX)return Fail(Status::InvalidInput,"Extra-node row references missing PART or SET",i);
    if(sets.First(x.source_node_set_id)!=i)return Fail(Status::DuplicateIdentity,"Repeated extra-node SET ID",i);
    if(s.parts[part].extra_row!=SIZE_MAX)return Fail(Status::DuplicateIdentity,"Multiple extra-node rows for one PART are unsupported",i);
    s.parts[part].extra_row=i;s.extras[i]={x.source_node_set_id,part,offset,x.node_count};
    std::copy_n(x.nodes,x.node_count,s.original.get()+offset);offset+=x.node_count;
  }
  return {};
}
template<class Storage> Report BuildRoots(const PartTopologyInput& in,Storage& s,const Index& parts) {
  // Source-sized, exact arrays; counted in the preflight scratch budget.
  auto child=std::make_unique<std::size_t[]>(in.part_count);
  auto parent=std::make_unique<std::size_t[]>(in.part_count);
  std::fill_n(child.get(),in.part_count,SIZE_MAX);std::fill_n(parent.get(),in.part_count,SIZE_MAX);
  for(std::size_t i=0;i<in.merge_count;++i) {
    const auto m=in.merges[i];const auto p=parts.First(m.parent_part_id),c=parts.First(m.child_part_id);
    if(p==SIZE_MAX||c==SIZE_MAX||p==c)return Fail(Status::UnsupportedMerge,"Merge references missing or identical PARTs",i);
    if(child[p]!=SIZE_MAX||parent[c]!=SIZE_MAX||parent[p]!=SIZE_MAX||child[c]!=SIZE_MAX)
      return Fail(Status::UnsupportedMerge,"Only disjoint one-child rigid merges are admitted",i);
    child[p]=c;parent[c]=p;s.merges[i]=m;
  }
  std::size_t offset=0,root=0;
  for(std::size_t p=0;p<in.part_count;++p) {
    if(parent[p]!=SIZE_MAX)continue;
    auto& out=s.roots[root];out={p,child[p],offset,0,child[p]==SIZE_MAX?1u:2u};
    const auto append=[&](std::size_t index) {
      auto& part=s.parts[index];part.root_index=root;
      std::copy_n(s.original.get()+part.member_offset,part.member_count,s.root_members.get()+offset);
      offset+=part.member_count;
      if(part.extra_row!=SIZE_MAX) {
        const auto& extra=s.extras[part.extra_row];
        std::copy_n(s.original.get()+extra.member_offset,extra.member_count,s.root_members.get()+offset);
        offset+=extra.member_count;
      }
    };
    append(p);if(child[p]!=SIZE_MAX)append(child[p]);
    out.member_count=offset-out.member_offset;++root;
  }
  return {};
}
} // namespace topology_detail
} // namespace tl::fea::rigid
