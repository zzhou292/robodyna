// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Internal.h"
#include "../../self_contact_filters/Environment.h"
namespace tlfea::contact::radioss_type25::surface_interface::detail {
namespace {
bool Contains(const std::uint32_t* nodes,unsigned count,std::uint32_t value) {
  for(unsigned k=0;k<count;++k)if(nodes[k]==value)return true;
  return false;
}
}
Report Prepare(const Input& in,source::Work work,Vector* points) noexcept {
  if(!self_contact_filters::CompatibleHostArithmetic())return {Status::UnsupportedArithmetic};
  source_surfaces::Counts unused;
  const auto prepared=source::Prepare(in.physical,work,unused);
  if(prepared.status!=source_surfaces::Status::Ok)return FromPhysical(prepared);
  for(std::size_t i=0;i<in.physical.node_count;++i) {
    const auto value=in.positions.at(static_cast<std::uint32_t>(i));
    if(!startup::coating_detail::Finite(value))return {Status::InvalidInput,SIZE_MAX,SIZE_MAX,i};
    points[i]=value;
    if(in.coordinates==startup::Coordinates::Si)
      points[i]={value.x/in.units.length_m,value.y/in.units.length_m,value.z/in.units.length_m};
    if(!startup::coating_detail::Finite(points[i]))return {Status::NonfiniteResult,SIZE_MAX,SIZE_MAX,i};
  }
  for(std::size_t i=0;i<in.raw_face_count;++i) {
    const auto& face=in.raw_faces[i];const auto row=face.source.reader_row;
    const bool triangle=face.nodes[2]==face.nodes[3];
    const unsigned corners=triangle?3:4;
    for(unsigned k=0;k<corners;++k) {
      if(face.nodes[k]>=in.physical.node_count)return {Status::InvalidInput,i,SIZE_MAX,face.nodes[k]};
      for(unsigned j=0;j<k;++j)if(face.nodes[k]==face.nodes[j])return {Status::UnsupportedProfile,i};
    }
    const std::uint32_t* original=nullptr;
    unsigned slots=0;
    if(face.source.kind==source_surfaces::ParentKind::Solid) {
      if(face.raw_role!=1||!face.source.solid_face||face.source.solid_face>6||row>=in.physical.solid_count)
        return {Status::InvalidInput,i};
      const auto& source=in.physical.solids[row];
      if(source.element_id!=face.source.element_id||source.part_id!=face.source.part_id)return {Status::InvalidInput,i};
      original=source.nodes;slots=8;
    } else {
      const bool tri=face.source.kind==source_surfaces::ParentKind::ShellTriangle;
      if((!tri&&face.source.kind!=source_surfaces::ParentKind::ShellQuad)||triangle!=tri||
          face.raw_role!=(tri?7:3)||face.source.solid_face||
          row>=(tri?in.physical.triangle_count:in.physical.quad_count))return {Status::InvalidInput,i};
      const auto& source=(tri?in.physical.triangles:in.physical.quads)[row];
      if(source.element_id!=face.source.element_id||source.part_id!=face.source.part_id)return {Status::InvalidInput,i};
      original=source.nodes;slots=tri?3:4;
    }
    for(unsigned k=0;k<corners;++k)if(!Contains(original,slots,face.nodes[k]))return {Status::InvalidInput,i};
  }
  return {Status::Ok};
}
}
