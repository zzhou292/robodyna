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
  const bool mixed=role_policy::Mixed(in.topology);
  if(mixed) {
    for(std::size_t raw=0;raw<in.raw_origin_count;++raw) {
      const auto primary=in.raw_origin_to_primary[raw];
      const auto& origin=in.raw_origins[raw];
      if(primary>=in.primary_count || origin.origin!=PrimaryOrigin::SingleSourceFace ||
          origin.origin_count!=1 || !origin.physical_parent_id)return {Status::InvalidInput};
      if(origin.kind==PrimaryFaceKind::Shell) {
        if(origin.local_face)return {Status::InvalidInput,primary};
      } else if(origin.kind==PrimaryFaceKind::Solid) {
        if(!origin.local_face || origin.local_face>6)return {Status::InvalidInput,primary};
      } else return {Status::InvalidInput,primary};
      const auto& identity=in.primary_identities[primary];
      if(identity.kind!=origin.kind)return {Status::InvalidInput,primary};
      if(identity.origin==PrimaryOrigin::SingleSourceFace &&
          (identity.physical_parent_id!=origin.physical_parent_id || identity.local_face!=origin.local_face))
        return {Status::InvalidInput,primary};
      ++faces[primary].count;
      data.raw_origins[raw]=origin;
      data.raw_origin_to_primary[raw]=primary;
    }
  }
  std::size_t shell_count=0;
  for (std::size_t i=0;i<in.primary_count;++i) {
    const auto& face=in.primary[i];
    if(mixed) {
      const auto& identity=in.primary_identities[i];
      if(identity.origin_count!=faces[i].count)return {Status::InvalidInput,i};
      if(identity.origin==PrimaryOrigin::SingleSourceFace) {
        if(identity.origin_count!=1 || !face.source_id || identity.physical_parent_id!=face.source_id)
          return {Status::InvalidInput,i};
      } else if(identity.origin==PrimaryOrigin::MultipleOrigins) {
        if(identity.origin_count<2 || identity.physical_parent_id || identity.local_face || face.source_id)
          return {Status::InvalidInput,i};
      } else return {Status::InvalidInput,i};
      if(identity.kind==PrimaryFaceKind::Shell) {
        if(identity.local_face)return {Status::InvalidInput,i};
        ++shell_count;
      } else if(identity.kind==PrimaryFaceKind::Solid) {
        if((identity.origin==PrimaryOrigin::SingleSourceFace && (!identity.local_face || identity.local_face>6)) ||
            face.side_role!=ShellSideRole::Ordinary)return {Status::InvalidInput,i};
      } else return {Status::InvalidInput,i};
    }
    if (!mixed && !face.source_id) return {Status::InvalidInput,i};
    if (!role_policy::Valid(face.side_role) ||
        (!role_policy::StoresRoles(in.topology) && face.side_role != ShellSideRole::Ordinary))
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
  if(mixed && shell_count!=in.shell_primary_count)return {Status::InvalidInput};
  const auto identity_less=[&](Identity a,Identity b) {
    if(a.id!=b.id)return a.id<b.id;
    if(mixed) {
      const auto& x=in.primary_identities[a.ordinal];
      const auto& y=in.primary_identities[b.ordinal];
      if(x.kind!=y.kind)return x.kind<y.kind;
      if(x.local_face!=y.local_face)return x.local_face<y.local_face;
    }
    return a.ordinal<b.ordinal;
  };
  std::sort(ids,ids+in.primary_count,identity_less);
  duplicate=SIZE_MAX;
  for(std::size_t i=1;i<in.primary_count;++i) {
    if(ids[i-1].id!=ids[i].id)continue;
    if(mixed) {
      const auto& a=in.primary_identities[ids[i-1].ordinal];
      const auto& b=in.primary_identities[ids[i].ordinal];
      if(a.origin==PrimaryOrigin::MultipleOrigins && b.origin==PrimaryOrigin::MultipleOrigins)continue;
      if(a.kind!=b.kind || a.local_face!=b.local_face)continue;
    }
    duplicate=std::min(duplicate,std::size_t(ids[i].ordinal));
  }
  if(duplicate!=SIZE_MAX)return {Status::InvalidInput,duplicate};
  // Mixed I25SURFI has already filtered its full node+role keys. Different
  // retained source roles can legitimately share the same geometric nodes.
  if(!mixed) {
    std::sort(faces,faces+in.primary_count,FaceLess);
    for(std::size_t i=1;i<in.primary_count;++i)
      if(SameFace(faces[i-1],faces[i]))return {Status::UnsupportedTopology,faces[i].ordinal};
  }
  constexpr unsigned opposite[]{1,0,3,2};
  std::size_t appended=in.primary_count;
  for(std::size_t i=0;i<in.primary_count;++i) {
    const auto& source=in.primary[i];
    const bool solid=mixed && in.primary_identities[i].kind==PrimaryFaceKind::Solid;
    auto& first=data.mains[i];
    first.source_id=source.source_id;
    first.global_id=static_cast<int>(i+1);
    data.expanded_to_primary[i]=static_cast<std::uint32_t>(i);
    data.primary_to_partner[i]=0;
    if(data.primary_roles)data.primary_roles[i]=source.side_role;
    if(data.primary_identities)data.primary_identities[i]=in.primary_identities[i];
    for(unsigned k=0;k<4;++k) {
      const bool reversed=source.side_role==ShellSideRole::CoatingReversed;
      first.nodes[k]=source.nodes[reversed?opposite[k]:k];
    }
    if(solid) {
      first.segment_type=0;
      continue;
    }
    if(appended>=data.main_count)return {Status::InvalidInput,i};
    const auto partner=appended++;
    auto& second=data.mains[partner];
    second.source_id=source.source_id;
    second.global_id=static_cast<int>(partner+1);
    const int offset=source.side_role==ShellSideRole::Ordinary?0:static_cast<int>(data.main_count);
    first.segment_type=second.global_id+offset;
    second.segment_type=-(first.global_id+offset);
    data.expanded_to_primary[partner]=static_cast<std::uint32_t>(i);
    data.primary_to_partner[i]=static_cast<std::uint32_t>(partner+1);
    for(unsigned k=0;k<4;++k)second.nodes[k]=first.nodes[opposite[k]];
  }
  if(appended!=data.main_count)return {Status::InvalidInput};
  return {Status::Ok};
}
} // namespace tlfea::contact::radioss_type25::startup::detail
