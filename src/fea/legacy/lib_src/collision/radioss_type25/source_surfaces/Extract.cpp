// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Internal.h"
namespace tlfea::contact::radioss_type25::source_surfaces::detail {
namespace {
void RotateMinimum(const std::uint32_t* nodes,unsigned count,std::uint32_t* rotated) {
  unsigned at=0;
  for(unsigned k=1;k<count;++k)if(nodes[k]<nodes[at])at=k;
  for(unsigned k=0;k<count;++k)rotated[k]=nodes[(at+k)%count];
}
bool Contains(const std::uint32_t* all,unsigned count,std::uint32_t node) {
  for(unsigned k=0;k<count;++k)if(all[k]==node)return true;
  return false;
}
bool IsInternal(const Input& in,const Work& w,std::size_t row,const std::uint32_t* nodes,unsigned count) {
  std::uint32_t key[4];RotateMinimum(nodes,count,key);
  for(auto at=w.solid_offsets[key[0]];at<w.solid_offsets[key[0]+1];++at) {
    const auto other=w.solid_rows[at];
    if(other==row||!w.selected[other])continue;
    const auto& solid=in.solids[other];bool all=true;
    for(unsigned k=0;k<count;++k)all=all&&Contains(solid.nodes,8,key[k]);
    if(!all)continue;
    for(unsigned face=0;face<6;++face) {
      std::uint32_t candidate[4],rotated[4];const auto corners=CompactFace(solid,face,candidate);
      if(corners!=count)continue;
      RotateMinimum(candidate,count,rotated);
      // Exact native oriented-edge criterion after the all-node solid test;
      // do not replace it by generic unordered graphics-face cancellation.
      if(rotated[0]==key[0]&&rotated[count-1]==key[1])return true;
    }
  }
  return false;
}
bool Suppressed(const Input& in,const Work& w,const std::uint32_t* nodes,unsigned count) {
  const auto* offsets=count==3?w.triangle_offsets:w.quad_offsets;
  const auto* rows=count==3?w.triangle_rows:w.quad_rows;
  const auto* shells=count==3?in.triangles:in.quads;
  for(auto at=offsets[nodes[0]];at<offsets[nodes[0]+1];++at) {
    const auto& shell=shells[rows[at]];unsigned equal=0;
    for(unsigned i=0;i<count;++i)for(unsigned j=0;j<count;++j)equal+=nodes[i]==shell.nodes[j]?1:0;
    if(equal==count)return SelectedPart(in,w,shell.part_id); // First match, even if unselected.
  }
  return false;
}
void AddShell(const Shell& shell,ParentKind kind,std::size_t row,bool reverse,Work w,std::size_t& count) {
  Face value;value.source={kind,shell.element_id,shell.part_id,std::uint32_t(row),0};
  const bool tri=kind==ParentKind::ShellTriangle;const unsigned slots=tri?3:4;
  for(unsigned k=0;k<slots;++k)value.nodes[k]=shell.nodes[reverse?slots-k-1:k];
  if(tri)value.nodes[3]=value.nodes[2];
  value.raw_role=tri?7:3;value.buffer_ordinal=std::uint32_t(count);w.faces[count++]=value;
}
bool Less(const Face& a,const Face& b) {
  for(unsigned k=0;k<4;++k)if(a.nodes[k]!=b.nodes[k])return a.nodes[k]<b.nodes[k];
  if(a.source.reader_row!=b.source.reader_row)return a.source.reader_row<b.source.reader_row;
  return a.buffer_ordinal<b.buffer_ordinal;
}
}
Report Extract(const Input& in,Work w,Counts& counts,std::size_t& count) noexcept {
  if(in.clause.mode!=SurfaceMode::All)
    for(std::size_t row=0;row<in.solid_count;++row)if(w.selected[row])
      for(unsigned face=0;face<6;++face) {
        std::uint32_t nodes[4];const auto corners=CompactFace(in.solids[row],face,nodes);
        if(corners<3)continue;
        if(IsInternal(in,w,row,nodes,corners))w.face_mask[row]=std::uint8_t(w.face_mask[row]+(1u<<face));
      }
  for(std::size_t row=0;row<in.solid_count;++row)if(w.selected[row])
    for(unsigned face=0;face<6;++face) {
      if(w.face_mask[row]&(1u<<face)){++counts.internal_faces;continue;}
      std::uint32_t nodes[4];const auto corners=CompactFace(in.solids[row],face,nodes);
      if(corners<3){++counts.degenerate_faces;continue;}
      if(Suppressed(in,w,nodes,corners)){++counts.shell_suppressed_faces;continue;}
      Face value;value.source={ParentKind::Solid,in.solids[row].element_id,in.solids[row].part_id,
          std::uint32_t(row),std::uint8_t(face+1)};
      for(unsigned k=0;k<corners;++k)value.nodes[k]=nodes[k];
      if(corners==3)value.nodes[3]=nodes[2];
      value.raw_role=1;value.buffer_ordinal=std::uint32_t(count);w.faces[count++]=value;
      w.surface_flags[row]=1;++counts.solid_faces;
    }
  for(std::size_t row=0;row<in.quad_count;++row)if(SelectedPart(in,w,in.quads[row].part_id)) {
    AddShell(in.quads[row],ParentKind::ShellQuad,row,in.clause.reverse_shell_normals,w,count);++counts.shell_faces;
  }
  for(std::size_t row=0;row<in.triangle_count;++row)if(SelectedPart(in,w,in.triangles[row].part_id)) {
    AddShell(in.triangles[row],ParentKind::ShellTriangle,row,in.clause.reverse_shell_normals,w,count);++counts.shell_faces;
  }
  // CREATE_SURFACE sorts four represented one-based node IDs then family-local
  // ELEM index; +1 does not change order. Stable source insertion resolves ties.
  if(count)std::sort(w.faces,w.faces+count,Less);
  return {Status::Ok,SIZE_MAX,SIZE_MAX,count,true};
}
}
