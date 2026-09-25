// SPDX-License-Identifier: AGPL-3.0-or-later
// SH2SURF25 ordinary and explicitly resolved coated shell branches; source order persists.
#include "Internal.h"
#include "lib_src/math/Fixed3Operations.h"
#include <algorithm>
namespace tlfea::contact::radioss_type25::startup::detail {
namespace {
bool IdentityLess(Identity a,Identity b) noexcept {
  return a.id<b.id || (a.id==b.id && a.ordinal<b.ordinal);
}
bool FaceLess(const FaceKey& a,const FaceKey& b) noexcept {
  if (a.count!=b.count) return a.count<b.count;
  for (unsigned k=0;k<a.count;++k) if (a.nodes[k]!=b.nodes[k]) return a.nodes[k]<b.nodes[k];
  return a.ordinal<b.ordinal;
}
bool SameFace(const FaceKey& a,const FaceKey& b) noexcept {
  if (a.count!=b.count) return false;
  for (unsigned k=0;k<a.count;++k) if (a.nodes[k]!=b.nodes[k]) return false;
  return true;
}
}
Report Expand(const Input& in,Data data,Vector* points,Identity* ids,FaceKey* faces) noexcept {
  for (std::size_t i=0;i<in.node_count;++i) {
    if (!in.node_source_ids[i]) return {Status::InvalidInput,SIZE_MAX,i};
    ids[i]={in.node_source_ids[i],static_cast<std::uint32_t>(i)};
    const auto p=in.positions.at(static_cast<std::uint32_t>(i));
    points[i]={p.x,p.y,p.z};
    if (!tl::math::Finite(p.x)||!tl::math::Finite(p.y)||!tl::math::Finite(p.z))
      return {Status::InvalidInput,SIZE_MAX,i};
    if (in.coordinates==Coordinates::Si) points[i]={p.x/in.units.length_m,p.y/in.units.length_m,p.z/in.units.length_m};
    if (!tl::math::Finite(points[i].x)||!tl::math::Finite(points[i].y)||!tl::math::Finite(points[i].z))
      return {Status::NonfiniteResult,SIZE_MAX,i};
  }
  std::sort(ids,ids+in.node_count,IdentityLess);
  std::size_t duplicate=SIZE_MAX;
  for (std::size_t i=1;i<in.node_count;++i)
    if (ids[i-1].id==ids[i].id) duplicate=std::min(duplicate,std::size_t(ids[i].ordinal));
  if (duplicate!=SIZE_MAX) return {Status::InvalidInput,SIZE_MAX,duplicate};
  for (std::size_t i=0;i<in.primary_count;++i) {
    const auto& face=in.primary[i];
    if (!face.source_id) return {Status::InvalidInput,i};
    if (!role_policy::Valid(face.side_role) ||
        (!role_policy::Resolved(in.topology) && face.side_role != ShellSideRole::Ordinary))
      return {Status::UnsupportedProfile,i};
    ids[i]={face.source_id,static_cast<std::uint32_t>(i)};
    const unsigned count=face.layout==ShellLayout::Triangle3?3:4;
    if (face.layout!=ShellLayout::Triangle3 && face.layout!=ShellLayout::Quad4)
      return {Status::UnsupportedProfile,i};
    if (count==3 && face.nodes[2]!=face.nodes[3]) return {Status::InvalidInput,i};
    faces[i].count=count; faces[i].ordinal=static_cast<std::uint32_t>(i);
    for (unsigned k=0;k<4;++k) {
      if (face.nodes[k]>=in.node_count) return {Status::InvalidInput,i,face.nodes[k]};
      faces[i].nodes[k]=face.nodes[k];
    }
    std::sort(faces[i].nodes,faces[i].nodes+count);
    for (unsigned k=1;k<count;++k)
      if (faces[i].nodes[k-1]==faces[i].nodes[k]) return {Status::UnsupportedTopology,i};
  }
  std::sort(ids,ids+in.primary_count,IdentityLess); duplicate=SIZE_MAX;
  for (std::size_t i=1;i<in.primary_count;++i)
    if (ids[i-1].id==ids[i].id) duplicate=std::min(duplicate,std::size_t(ids[i].ordinal));
  if (duplicate!=SIZE_MAX) return {Status::InvalidInput,duplicate};
  std::sort(faces,faces+in.primary_count,FaceLess);
  for (std::size_t i=1;i<in.primary_count;++i)
    if (SameFace(faces[i-1],faces[i])) return {Status::UnsupportedTopology,faces[i].ordinal};
  constexpr unsigned opposite[]{1,0,3,2};
  for (std::size_t i=0;i<in.primary_count;++i) {
    const auto& source=in.primary[i]; const auto partner=in.primary_count+i;
    auto& first=data.mains[i]; auto& second=data.mains[partner];
    first.source_id=source.source_id; second.source_id=source.source_id;
    first.global_id=static_cast<int>(i+1); second.global_id=static_cast<int>(partner+1);
    // MakeLayout bounds8P before any conversion: encoded coating magnitude is
    // at most4P, and the reference count remains the stronger integer bound.
    const int offset = source.side_role == ShellSideRole::Ordinary ? 0 : static_cast<int>(2*in.primary_count);
    first.segment_type=second.global_id+offset;
    second.segment_type=-(first.global_id+offset);
    if (data.primary_roles) data.primary_roles[i]=source.side_role;
    data.expanded_to_primary[i]=data.expanded_to_primary[partner]=static_cast<std::uint32_t>(i);
    data.primary_to_partner[i]=static_cast<std::uint32_t>(partner+1);
    for (unsigned k=0;k<4;++k) {
      const bool reversed = source.side_role == ShellSideRole::CoatingReversed;
      first.nodes[k]=source.nodes[reversed?opposite[k]:k];
      second.nodes[k]=source.nodes[reversed?k:opposite[k]];
    }
  }
  return {Status::Ok};
}
} // namespace tlfea::contact::radioss_type25::startup::detail
