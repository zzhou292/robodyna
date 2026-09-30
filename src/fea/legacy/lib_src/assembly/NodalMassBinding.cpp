// SPDX-License-Identifier: AGPL-3.0-or-later
#include "NodalMassBinding.h"
#include "../elements/type25/Type25Model.h"
#include "../../lib_utils/BoundedArena.h"
#include <cmath>
#include <cstring>
#include <new>
#include <stdexcept>

namespace tl::fea {
namespace {
using S=NodalMassStatus;
bool Same(double a,double b) noexcept { return std::memcmp(&a,&b,sizeof(a))==0; }
bool Same(tl::math::Vec3 a,tl::math::Vec3 b) noexcept {
  return Same(a.x,b.x)&&Same(a.y,b.y)&&Same(a.z,b.z);
}
bool Add(double& sum,double value) noexcept {
  sum+=value; return std::isfinite(sum)&&sum>=0;
}
}
struct NodalMassBinding::Impl {
  ShellBatchBinding shells;
  type25::Model connectors;
  tl::util::BoundedStartupArray<NodalMassNode,0> nodes;
  NodalMassPartitions totals;
  std::size_t bytes=0;
  Impl(const ShellBatchBinding& s,const type25::Model& c):shells(s),connectors(c) {}
};

NodalMassReport NodalMassBinding::Initialize(const ShellBatchBinding& shells,
    const type25::Model& connectors,const NodalMassLimits& limits) noexcept try {
  if(impl_) return {S::AlreadyInitialized,"Combined nodal mass is immutable"};
  if(!shells.prepared()||!connectors.prepared()||!connectors.source_instance_id()||
     !connectors.connection_count()||connectors.global_node_count()!=shells.node_count())
    return {S::InvalidInput,"Complete prepared shell and connector models are required"};
  const auto count=shells.node_count();
  const auto hard=type25::Bounds(limits.profile);const bool vehicle=limits.profile==type25::CapacityProfile::Vehicle;
  if(!type25::ValidProfile(limits.profile)||!limits.max_nodes||limits.max_nodes>hard.nodes||count>limits.max_nodes||
     !limits.max_host_bytes||limits.max_host_bytes>hard.combined_host_bytes)
    return {S::ResourceLimit,"Combined nodal mass limits exceed bounded startup domain"};
  // Legacy conservative accounting remains unchanged. Vehicle counts embedded
  // handle bytes through Impl and retains each actual producer backing once.
  // Qualified producer models already validated their original input extents.
  std::size_t shell_bytes=shells.host_bytes(),connector_bytes=connectors.owned_payload_bytes();
  if(vehicle) {
    if(shell_bytes<sizeof(ShellBatchBinding)||connector_bytes<sizeof(type25::Model))
      return {S::ResourceLimit,"TYPE25 retained producer payload is inconsistent"};
    shell_bytes-=sizeof(ShellBatchBinding);connector_bytes-=sizeof(type25::Model);
  }
  util::BoundedArenaLayout budget(limits.max_host_bytes); util::ArenaRegion ignored;
  if(!budget.Append<unsigned char>(sizeof(Impl)+64,ignored)||
     !budget.Append<NodalMassNode>(count,ignored)||!budget.Append<unsigned char>(64,ignored)||
     !budget.Append<unsigned char>(shell_bytes,ignored)||
     !budget.Append<unsigned char>(connector_bytes,ignored))
    return {S::ResourceLimit,"Combined nodal mass payload exceeds startup budget"};
  auto next=std::make_shared<Impl>(shells,connectors);
  next->nodes.Resize(count); next->bytes=budget.bytes(); next->totals.shell=shells.totals();
  for(std::size_t n=0;n<count;++n) {
    const auto& source=shells.nodes()[n]; auto& node=next->nodes[n];
    node.source_id=source.source_id; node.position=source.position;
    node.coefficients.shell=source.native;
    node.coefficients.mass=source.native.mass;
    node.coefficients.isotropic_inertia=source.native.isotropic_inertia;
  }
  for(std::size_t c=0;c<connectors.connection_count();++c) {
    const auto& connection=connectors.connections()[c];
    for(unsigned endpoint=0;endpoint<2;++endpoint) {
      const auto& contribution=connectors.endpoint_mass()[2*c+endpoint];
      const auto n=connection.global_node[endpoint];
      if(n>=count||contribution.global_node!=n||
         contribution.source_node_id!=connection.source_node_id[endpoint]||
         contribution.source_element_id!=connection.source_element_id||
         contribution.source_property_id!=connectors.properties()[connection.property_index].source_property_id||
         next->nodes[n].source_id!=connection.source_node_id[endpoint])
        return {S::IdentityMismatch,"Connector endpoint differs from complete shell node inventory",n,c};
      if(!Same(next->nodes[n].position,connection.position[endpoint]))
        return {S::PositionMismatch,"Connector endpoint reference coordinate bits differ",n,c};
      auto& values=next->nodes[n].coefficients;
      if(!std::isfinite(contribution.mass_kg)||contribution.mass_kg<=0||
         !std::isfinite(contribution.isotropic_inertia_kg_m2)||contribution.isotropic_inertia_kg_m2<=0||
         !Add(values.connector_mass,contribution.mass_kg)||
         !Add(values.connector_inertia,contribution.isotropic_inertia_kg_m2)||
         !Add(values.mass,contribution.mass_kg)||
         !Add(values.isotropic_inertia,contribution.isotropic_inertia_kg_m2)||
         !Add(next->totals.connector_mass,contribution.mass_kg)||
         !Add(next->totals.connector_inertia,contribution.isotropic_inertia_kg_m2))
        return {S::NonfiniteResult,"Connector mass/inertia accumulation is not finite positive",n,c};
    }
  }
  for(std::size_t n=0;n<count;++n) {
    const auto& values=next->nodes[n].coefficients;
    if(!std::isfinite(values.mass)||values.mass<=0||!std::isfinite(1./values.mass)||
       !std::isfinite(values.isotropic_inertia)||values.isotropic_inertia<=0||
       !std::isfinite(1./values.isotropic_inertia)||
       !Add(next->totals.mass,values.mass)||!Add(next->totals.isotropic_inertia,values.isotropic_inertia))
      return {S::NonfiniteResult,"Combined nodal coefficients cannot supply finite inverse M/J",n};
  }
  impl_=std::move(next); return {S::Success,"OK"};
} catch(const std::bad_alloc&) { return {S::ResourceLimit,"Combined nodal mass allocation failed"}; }
  catch(const std::length_error&) { return {S::ResourceLimit,"Combined nodal mass size overflow"}; }

std::size_t NodalMassBinding::node_count() const noexcept { return impl_?impl_->nodes.size():0; }
std::uint64_t NodalMassBinding::source_instance_id() const noexcept {
  return impl_?impl_->connectors.source_instance_id():0;
}
std::size_t NodalMassBinding::host_bytes() const noexcept { return impl_?impl_->bytes:0; }
tl::util::ConstView<NodalMassNode> NodalMassBinding::nodes() const noexcept {
  static const NodalMassNode empty;
  return {impl_?impl_->nodes.data():&empty,node_count()};
}
const NodalMassPartitions& NodalMassBinding::totals() const noexcept {
  static const NodalMassPartitions empty; return impl_?impl_->totals:empty;
}
bool NodalMassBinding::Matches(const ShellBatchBinding& shells) const noexcept {
  return impl_&&shells.prepared()&&impl_->shells.inventory()==shells.inventory();
}
bool NodalMassBinding::Matches(const type25::Model& connectors) const noexcept {
  return impl_&&impl_->connectors.Matches(connectors);
}
bool NodalMassBinding::SharesConnectorStorage(const type25::Model& connectors) const noexcept {
  return impl_&&impl_->connectors.SharesStorage(connectors);
}
bool NodalMassBinding::Matches(const NodalMassBinding& other) const noexcept {
  return impl_&&other.impl_&&(impl_==other.impl_||
      (Matches(other.impl_->shells)&&Matches(other.impl_->connectors)));
}
} // namespace tl::fea
