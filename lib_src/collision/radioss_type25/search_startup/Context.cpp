// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Internal.h"
#include <climits>
namespace tlfea::contact::radioss_type25::search_startup::detail {
Report ResolveContext(const Input& in,Limits limits,Context context,std::size_t& native_nodes) noexcept {
  const auto& c=in.contributors;
  if(c.census!=Census::CompleteDeclaredModel||c.physical_nodes!=in.mesh.node_count||
      c.physical_shells<in.mesh.primary_count||c.other_interfaces||c.unsupported_elements)
    return {Status::UnsupportedProfile};
  if(c.physical_shells>std::size_t(INT_MAX)||c.rigid_bodies>std::size_t(INT_MAX)||
      c.tied_interfaces>std::size_t(INT_MAX)||c.cin_links>std::size_t(INT_MAX))return {Status::ResourceLimit};
  if(c.native_auxiliary_nodes!=in.auxiliary_rigid_primary_count)
    return {Status::InvalidInput};
  // This explicit extension covers one generated native primary per declared
  // rigid body. Other auxiliary populations need their own source contract.
  if(c.native_auxiliary_nodes!=c.rigid_bodies)return {Status::UnsupportedProfile};
  if(context==Context::LegacyNoKinematics&&(c.tied_interfaces||c.rigid_bodies||c.cin_links))
    return {Status::UnsupportedProfile};
  if(context==Context::RigidOnly&&(!c.rigid_bodies||c.tied_interfaces||c.cin_links))
    return {Status::UnsupportedProfile};
  if(context==Context::BeforeTied&&!c.tied_interfaces)return {Status::UnsupportedProfile};
  if(context!=Context::LegacyNoKinematics&&context!=Context::RigidOnly&&context!=Context::BeforeTied)
    return {Status::UnsupportedProfile};
  if(!limits.max_native_model_nodes||limits.max_native_model_nodes>std::size_t(INT_MAX)||
      c.physical_nodes>limits.max_native_model_nodes||
      c.native_auxiliary_nodes>limits.max_native_model_nodes-c.physical_nodes)
    return {Status::ResourceLimit};
  native_nodes=c.physical_nodes+c.native_auxiliary_nodes;
  return {Status::Ok};
}
Report CheckAuxiliaryIds(const Input& in,Work work) noexcept {
  const auto count=in.auxiliary_rigid_primary_count;
  if(!count)return {Status::Ok};
  std::copy_n(in.auxiliary_rigid_primary_ids,count,work.auxiliary_ids);
  std::sort(work.auxiliary_ids,work.auxiliary_ids+count);
  for(std::size_t i=0;i<count;++i)
    if(!work.auxiliary_ids[i]||work.auxiliary_ids[i]>std::uint64_t(INT_MAX)||
        (i&&work.auxiliary_ids[i]==work.auxiliary_ids[i-1]))return {Status::InvalidInput,SIZE_MAX,i};
  // Sort only the small auxiliary list; do not scan every physical node for
  // every rigid body, or change the source geometry/reference order.
  for(std::size_t i=0;i<in.mesh.node_count;++i)
    if(std::binary_search(work.auxiliary_ids,work.auxiliary_ids+count,in.mesh.node_source_ids[i]))
      return {Status::InvalidInput,SIZE_MAX,i};
  return {Status::Ok};
}
}
