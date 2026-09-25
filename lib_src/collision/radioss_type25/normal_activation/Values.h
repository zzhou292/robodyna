// SPDX-License-Identifier: AGPL-3.0-or-later
// Derived from OpenRadioss, Copyright (C) 2026 Siemens; see ../LICENSE.md.
// Selected I25TAGN / I25FREE_BOUND, pinned OpenRadioss a62b27e6.
#pragma once
#include "Types.h"
namespace tlfea::contact::radioss_type25::normal_activation::detail {
TL_MATH_HOST_DEVICE inline bool FreeMain(const lifecycle::Main& main) {
  if(main.coefficient<=0)return false;
  for(unsigned j=0;j<4;++j)
    if(main.neighbors[j]==0&&!(j==2&&main.nodes[2]==main.nodes[3]))return true;
  return false;
}
TL_MATH_HOST_DEVICE inline int Partner(int role,std::size_t mains) {
  std::int64_t id=role;if(id<0)id=-id;
  if(id>std::int64_t(mains))id-=std::int64_t(mains);
  return int(id);
}
// Writers implement Main(zero_based_index) and Node(zero_based_index). A CUDA
// parallel writer uses integer OR/Exch only; the union is order independent.
template<class Writer>
TL_MATH_HOST_DEVICE inline void MainAndNodes(const lifecycle::SourceView& source,
    std::uint32_t main,Writer& write) {
  write.Main(main);for(auto node:source.mains[main].nodes)write.Node(node);
}
template<class Writer>
TL_MATH_HOST_DEVICE inline void RetainedRow(const Input& in,std::size_t row,Writer& write) {
  const auto& history=in.rows[row].stage.value.history.row;
  if(history.irtlm[0]<=0||in.source.secondary[row].coefficient==0||
     history.irtlm[3]!=in.profile.local_processor)return;
  const auto local=std::uint32_t(history.irtlm[2]-1);write.Main(local);
  const auto& incidence=in.source.normal_to_main;
  const auto& removed=in.source.removed_main_by_secondary;
  for(auto reference:in.source.mains[local].normal_reference)
    for(auto i=incidence.offsets[reference-1];i<incidence.offsets[reference];++i) {
      const auto neighbor=incidence.entries[i];
      if(in.source.mains[neighbor-1].coefficient>0&&
         !lifecycle::detail::Contains(removed.entries,removed.offsets[row],removed.offsets[row+1],neighbor))
        MainAndNodes(in.source,neighbor-1,write);
    }
}
template<class Writer>
TL_MATH_HOST_DEVICE inline void OptimizedMain(const Input& in,std::size_t item,Writer& write) {
  const auto local=in.optimized_main_ids[item]-1;MainAndNodes(in.source,local,write);
  const int role=in.source.mains[local].segment_type;
  if(role>0)write.Main(std::uint32_t(Partner(role,in.source.main_count)-1));
}
template<class Writer>
TL_MATH_HOST_DEVICE inline void FreeMain(const Input& in,std::size_t item,Writer& write) {
  const auto local=in.free_main_ids[item]-1;write.Main(local);
  const auto& main=in.source.mains[local];const int partner=Partner(main.segment_type,in.source.main_count);
  if(partner)write.Main(std::uint32_t(partner-1));
  for(unsigned j=0;j<4;++j) {
    if(main.neighbors[j]!=0||(j==2&&main.nodes[2]==main.nodes[3]))continue;
    write.Node(main.nodes[j]);write.Node(main.nodes[(j+1)%4]);
  }
}
} // namespace tlfea::contact::radioss_type25::normal_activation::detail
