// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Internal.h"
namespace tlfea::contact::radioss_type25::source_surfaces::detail {
namespace {
template<class T> void Incidence(const T* rows,std::size_t count,unsigned slots,
    std::size_t nodes,std::uint32_t* offsets,std::uint32_t* entries,std::uint32_t* cursor) {
  std::fill_n(offsets,nodes+1,0);
  for(unsigned k=0;k<slots;++k)for(std::size_t i=0;i<count;++i)++offsets[rows[i].nodes[k]+1];
  for(std::size_t n=0;n<nodes;++n)offsets[n+1]+=offsets[n];
  std::copy_n(offsets,nodes,cursor);
  // Original BUILD_CNEL corner-major insertion, including repeated raw8 slots.
  for(unsigned k=0;k<slots;++k)for(std::size_t i=0;i<count;++i)
    entries[cursor[rows[i].nodes[k]]++]=std::uint32_t(i);
}
Report CheckShell(const Shell& row,unsigned slots,std::size_t ordinal,std::size_t nodes) {
  if(!row.element_id||!row.part_id||(slots==3&&row.nodes[3]!=row.nodes[2]))return {Status::InvalidInput,ordinal};
  for(unsigned i=0;i<slots;++i) {
    if(row.nodes[i]>=nodes)return {Status::InvalidInput,ordinal,row.nodes[i]};
    for(unsigned j=0;j<i;++j)if(row.nodes[i]==row.nodes[j])return {Status::UnsupportedProfile,ordinal};
  }
  return {Status::Ok};
}
}
bool SelectedPart(const Input& in,const Work& w,std::uint64_t id) noexcept {
  return in.clause.kind==ClauseKind::Parts&&in.clause.part_count&&
      std::binary_search(w.parts,w.parts+in.clause.part_count,id);
}
Report Prepare(const Input& in,Work w,Counts& counts) noexcept {
  if(in.clause.part_count) {
    std::copy_n(in.clause.part_ids,in.clause.part_count,w.parts);
    std::sort(w.parts,w.parts+in.clause.part_count);
    for(std::size_t i=0;i<in.clause.part_count;++i)
      if(!w.parts[i]||(i&&w.parts[i]==w.parts[i-1]))return {Status::InvalidInput,i};
  }
  if(in.solid_count) {
    std::fill_n(w.selected,in.solid_count,0);
    std::fill_n(w.face_mask,in.solid_count,0);
    std::fill_n(w.surface_flags,in.solid_count,0);
  }
  for(std::size_t i=0;i<in.solid_count;++i) {
    const auto& row=in.solids[i];
    if(!row.element_id||!row.part_id)return {Status::InvalidInput,i};
    if(row.topology!=SolidTopology::Hex8&&row.topology!=SolidTopology::DeclaredPenta6&&
        row.topology!=SolidTopology::NativeRaw8)
      return {Status::UnsupportedProfile,i};
    for(unsigned k=0;k<8;++k)if(row.nodes[k]>=in.node_count)return {Status::InvalidInput,i,row.nodes[k]};
    if(row.topology==SolidTopology::DeclaredPenta6&&(row.nodes[3]!=row.nodes[0]||row.nodes[7]!=row.nodes[4]))
      return {Status::InvalidInput,i};
    for(unsigned k=0;k<8 && row.topology!=SolidTopology::NativeRaw8;++k) {
      if(row.topology==SolidTopology::DeclaredPenta6&&(k==3||k==7))continue;
      for(unsigned j=0;j<k;++j) {
        if(row.topology==SolidTopology::DeclaredPenta6&&(j==3||j==7))continue;
        if(row.nodes[k]==row.nodes[j])return {Status::UnsupportedProfile,i};
      }
    }
    w.selected[i]=SelectedPart(in,w,row.part_id)?1:0;
  }
  if(in.clause.kind==ClauseKind::Solids)
    for(std::size_t i=0;i<in.clause.solid_row_count;++i) {
      const auto row=in.clause.solid_rows[i];
      if(row>=in.solid_count||(i&&row<=in.clause.solid_rows[i-1]))return {Status::InvalidInput,i};
      w.selected[row]=1;
    }
  for(std::size_t i=0;i<in.solid_count;++i)counts.selected_solids+=w.selected[i];
  for(std::size_t i=0;i<in.quad_count;++i) {
    const auto r=CheckShell(in.quads[i],4,i,in.node_count);if(r.status!=Status::Ok)return r;
    counts.selected_quads+=SelectedPart(in,w,in.quads[i].part_id)?1:0;
  }
  for(std::size_t i=0;i<in.triangle_count;++i) {
    const auto r=CheckShell(in.triangles[i],3,i,in.node_count);if(r.status!=Status::Ok)return r;
    counts.selected_triangles+=SelectedPart(in,w,in.triangles[i].part_id)?1:0;
  }
  Incidence(in.solids,in.solid_count,8,in.node_count,w.solid_offsets,w.solid_rows,w.cursor);
  Incidence(in.quads,in.quad_count,4,in.node_count,w.quad_offsets,w.quad_rows,w.cursor);
  Incidence(in.triangles,in.triangle_count,3,in.node_count,w.triangle_offsets,w.triangle_rows,w.cursor);
  return {Status::Ok};
}
}
