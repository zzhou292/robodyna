// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Internal.h"
#include "../../RadiossType25SearchStartup.h"
#include <climits>
namespace tlfea::contact::radioss_type25::search_startup::detail {
Report ResolveContext(const Input& in,Limits limits,Context context,std::size_t& native_nodes) noexcept {
  const auto& c=in.contributors;
  const bool composed=context==Context::ComposedNoTied||context==Context::ComposedBeforeTied;
  const bool mixed=composed&&in.mesh.profile==startup::Profile::MixedSurface;
  if(c.census!=Census::CompleteDeclaredModel||c.physical_nodes!=in.mesh.node_count||
      c.physical_shells<(mixed?in.mesh.shell_primary_count:in.mesh.primary_count)||(composed?c.other_interfaces!=in.covered_type25_siblings:(c.other_interfaces!=0||in.covered_type25_siblings!=0))||c.unsupported_elements)
    return {Status::UnsupportedProfile};
  if(c.physical_shells>std::size_t(INT_MAX)||c.rigid_bodies>std::size_t(INT_MAX)||
      c.tied_interfaces>std::size_t(INT_MAX)||c.cin_links>std::size_t(INT_MAX))return {Status::ResourceLimit};
  if(context==Context::LegacyNoKinematics&&(c.tied_interfaces||c.rigid_bodies||c.cin_links))
    return {Status::UnsupportedProfile};
  if(context==Context::RigidOnly&&(!c.rigid_bodies||c.tied_interfaces||c.cin_links))
    return {Status::UnsupportedProfile};
  if(context==Context::ComposedNoTied&&(c.tied_interfaces||c.cin_links))return {Status::UnsupportedProfile};
  if((context==Context::BeforeTied||context==Context::ComposedBeforeTied)&&!c.tied_interfaces)return {Status::UnsupportedProfile};
  if(context!=Context::LegacyNoKinematics&&context!=Context::RigidOnly&&context!=Context::BeforeTied&&context!=Context::ComposedNoTied&&context!=Context::ComposedBeforeTied)
    return {Status::UnsupportedProfile};
  if(!limits.max_native_model_nodes||limits.max_native_model_nodes>std::size_t(INT_MAX)||
      c.physical_nodes>limits.max_native_model_nodes)
    return {Status::ResourceLimit};
  const auto& population=in.native_population;
  if(population.policy==NativePopulationPolicy::CompleteModelMultiplierTier) {
    if(!composed)return {Status::UnsupportedProfile};
    // SIZE_MAX is an explicit unavailable exact auxiliary count. It cannot be
    // confused with the genuinely empty exact-ID legacy census.
    if(c.native_auxiliary_nodes!=SIZE_MAX||in.auxiliary_rigid_primary_ids||in.auxiliary_rigid_primary_count||
        population.lower<c.physical_nodes||!population.lower||population.upper<population.lower)
      return {Status::InvalidInput};
    if(population.upper>limits.max_native_model_nodes)return {Status::ResourceLimit};
    double lower=0,upper=0;
    if(ResolveMultiplier(population.lower,&lower)!=Status::Ok||ResolveMultiplier(population.upper,&upper)!=Status::Ok||lower!=upper)
      return {Status::UnsupportedProfile};
    native_nodes=population.lower; // Only the branch operand, never reported as exact NUMNOD.
    return {Status::Ok};
  }
  if(population.policy!=NativePopulationPolicy::ExactDeclaredAuxiliaryIds||population.lower||population.upper)
    return {Status::UnsupportedProfile};
  if(c.native_auxiliary_nodes!=in.auxiliary_rigid_primary_count)return {Status::InvalidInput};
  // The exact-ID profile covers one generated native primary per declared body.
  if(c.native_auxiliary_nodes!=c.rigid_bodies)return {Status::UnsupportedProfile};
  if(c.native_auxiliary_nodes>limits.max_native_model_nodes-c.physical_nodes)return {Status::ResourceLimit};
  native_nodes=c.physical_nodes+c.native_auxiliary_nodes;
  return {Status::Ok};
}
NativePopulation ResolvedPopulation(const Input& in,std::size_t multiplier_operand) noexcept {
  return in.native_population.policy==NativePopulationPolicy::ExactDeclaredAuxiliaryIds?
    NativePopulation{NativePopulationPolicy::ExactDeclaredAuxiliaryIds,multiplier_operand,multiplier_operand}:in.native_population;
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
