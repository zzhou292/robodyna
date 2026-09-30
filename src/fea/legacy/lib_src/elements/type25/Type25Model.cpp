// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Type25Model.h"
#include "Type25Frame.h"
#include "Type25Identity.h"
#include "Type25ModelIndex.h"
#include <algorithm>

namespace tl::fea::type25 {
struct Model::Impl {
  std::uint64_t source_instance_id=0;
  SourceUnits units{};
  std::size_t node_count=0,property_count=0,connection_count=0,owned_bytes=0,startup_bytes=0;
  std::unique_ptr<PropertyInput[]> properties;
  std::unique_ptr<ConnectionInput[]> connections;
  std::unique_ptr<Reference[]> references;
  std::unique_ptr<History[]> initial;
  std::unique_ptr<EndpointMass[]> mass;
};
namespace {
ModelReport Fail(Status status,const char* message,std::size_t index=SIZE_MAX) {
  return {status,message,index};
}
constexpr std::size_t SharedControlReserve=256;
bool Reserve(std::size_t count,std::size_t size,std::size_t cap,std::size_t& bytes) {
  if(bytes>cap||size==0||count>(cap-bytes)/size)return false;
  bytes+=count*size;return true;
}
ModelReport CheckConnection(const ModelInput& in,std::size_t i,const detail::ModelIndex* index) {
  const auto& c=in.connections[i];
  if(!c.source_element_id||c.property_index>=in.property_count||
     c.global_node[0]>=in.global_node_count||c.global_node[1]>=in.global_node_count||
     c.global_node[0]==c.global_node[1]||!c.source_node_id[0]||!c.source_node_id[1]||
     c.source_node_id[0]==c.source_node_id[1])return Fail(Status::InvalidInput,"Invalid connection identity or endpoint",i);
  if(index)return index->CheckPriorConnections(in,i);
  for(std::size_t j=0;j<i;++j) {
    const auto& prior=in.connections[j];
    if(prior.source_element_id==c.source_element_id)return Fail(Status::DuplicateIdentity,"Duplicate source connection ID",i);
    for(unsigned a=0;a<2;++a)for(unsigned b=0;b<2;++b) {
      const bool same_index=c.global_node[a]==prior.global_node[b];
      const bool same_id=c.source_node_id[a]==prior.source_node_id[b];
      if(same_index!=same_id||(same_index&&!detail::Same(c.position[a],prior.position[b])))
        return Fail(Status::DuplicateIdentity,"Shared endpoint source identity or exact coordinate mismatch",i);
    }
  }
  return {Status::Success,"",i};
}
} // namespace
Model::Model()=default;
Model::~Model()=default;
ModelReport Model::Initialize(const ModelInput& in) noexcept {
  if(impl_)return Fail(Status::InvalidInput,"TYPE25 model is already initialized");
  const auto& lim=in.limits;
  const auto hard=Bounds(lim.profile);const bool vehicle=lim.profile==CapacityProfile::Vehicle;
  if(!ValidProfile(lim.profile)||!lim.max_connections||lim.max_connections>hard.connections||!lim.max_properties||lim.max_properties>hard.properties||
     !lim.max_nodes||lim.max_nodes>hard.nodes||!lim.max_host_bytes||lim.max_host_bytes>hard.model_host_bytes||
     !in.connection_count||in.connection_count>lim.max_connections||!in.property_count||in.property_count>lim.max_properties||
     !in.global_node_count||in.global_node_count>lim.max_nodes)return Fail(Status::ResourceLimit,"TYPE25 count or hard limit exceeded");
  std::size_t bytes=sizeof(Model)+sizeof(Impl)+SharedControlReserve;
  if(!Reserve(in.property_count,sizeof(PropertyInput),lim.max_host_bytes,bytes)||
     !Reserve(in.connection_count,sizeof(ConnectionInput)+sizeof(Reference)+sizeof(History)+2*sizeof(EndpointMass),lim.max_host_bytes,bytes))
    return Fail(Status::ResourceLimit,"TYPE25 host payload exceeds declared budget");
  std::size_t startup_bytes=bytes;
  if(vehicle&&!Reserve(1,detail::ModelIndex::Bytes(in.property_count,in.connection_count),lim.max_host_bytes,startup_bytes))
    return Fail(Status::ResourceLimit,"TYPE25 vehicle identity scratch exceeds declared budget");
  detail::Units units;
  if(!in.source_instance_id||!in.properties||!in.connections||!detail::ResolveUnits(in.source_units,units))
    return Fail(Status::InvalidInput,"Missing source identity, units or input range");
  if(vehicle&&(reinterpret_cast<std::uintptr_t>(in.properties)>UINTPTR_MAX-in.property_count*sizeof(PropertyInput)||
      reinterpret_cast<std::uintptr_t>(in.connections)>UINTPTR_MAX-in.connection_count*sizeof(ConnectionInput)))
    return Fail(Status::InvalidInput,"Overflowing TYPE25 vehicle input range");
  try {
    auto next=std::make_shared<Impl>();
    next->source_instance_id=in.source_instance_id;next->units=in.source_units;next->node_count=in.global_node_count;
    next->owned_bytes=bytes;next->startup_bytes=startup_bytes;
    next->property_count=in.property_count;next->connection_count=in.connection_count;
    // One allocation per exact active array. Only explicit Vehicle startup
    // adds temporary indexes; the legacy path keeps its existing byte contract.
    next->properties=std::make_unique<PropertyInput[]>(in.property_count);
    std::copy_n(in.properties,in.property_count,next->properties.get());
    next->connections=std::make_unique<ConnectionInput[]>(in.connection_count);
    std::copy_n(in.connections,in.connection_count,next->connections.get());
    next->references=std::make_unique<Reference[]>(in.connection_count);
    next->initial=std::make_unique<History[]>(in.connection_count);
    next->mass=std::make_unique<EndpointMass[]>(2*in.connection_count);
    std::unique_ptr<detail::ModelIndex> index;
    if(vehicle){index=std::make_unique<detail::ModelIndex>();index->Prepare(in);}
    for(std::size_t p=0;p<in.property_count;++p) {
      if(!in.properties[p].source_property_id||!ValidProperty(in.properties[p].property))
        return Fail(Status::InvalidInput,"Invalid resolved TYPE25 property");
      if(index) {
        if(!index->FirstProperty(in,p))return Fail(Status::DuplicateIdentity,"Duplicate source property ID");
      } else for(std::size_t j=0;j<p;++j)if(in.properties[j].source_property_id==in.properties[p].source_property_id)
          return Fail(Status::DuplicateIdentity,"Duplicate source property ID");
    }
    for(std::size_t i=0;i<in.connection_count;++i) {
      const auto checked=CheckConnection(in,i,index.get());if(!checked)return checked;
      const auto& c=in.connections[i];const auto& p=in.properties[c.property_index];
      const auto frame=InitializeReference(in.source_units,c.position,c.seed,next->references[i]);
      if(frame!=Status::Success)return Fail(frame,"Invalid reference frame",i);
      next->initial[i].transverse_axis=next->references[i].transverse_axis;
      MassCoefficients coefficients;
      const auto status=EndpointCoefficients(p.property,coefficients);
      if(status!=Status::Success)return Fail(status,"Endpoint property mass or inertia is not representable",i);
      for(unsigned k=0;k<2;++k)next->mass[2*i+k]={c.source_element_id,p.source_property_id,c.source_node_id[k],c.global_node[k],
          coefficients.mass_kg,coefficients.isotropic_inertia_kg_m2};
    }
    impl_=std::move(next);return {Status::Success,"",SIZE_MAX};
  }catch(...){return Fail(Status::ResourceLimit,"TYPE25 startup allocation failed");}
}
bool Model::prepared() const noexcept { return static_cast<bool>(impl_); }
bool Model::SharesStorage(const Model& other) const noexcept {return impl_&&impl_==other.impl_;}
bool Model::Matches(const Model& other) const noexcept {
  if(!impl_||!other.impl_)return false;
  if(impl_==other.impl_)return true;
  if(source_instance_id()!=other.source_instance_id()||global_node_count()!=other.global_node_count()||
     property_count()!=other.property_count()||connection_count()!=other.connection_count()||
     !detail::Same(source_units(),other.source_units()))return false;
  for(std::size_t i=0;i<property_count();++i)if(!detail::Same(properties()[i],other.properties()[i]))return false;
  for(std::size_t i=0;i<connection_count();++i)if(!detail::Same(connections()[i],other.connections()[i]))return false;
  return true;
}
std::uint64_t Model::source_instance_id() const noexcept { return impl_?impl_->source_instance_id:0; }
SourceUnits Model::source_units() const noexcept { return impl_?impl_->units:SourceUnits{}; }
std::size_t Model::global_node_count() const noexcept { return impl_?impl_->node_count:0; }
std::size_t Model::connection_count() const noexcept { return impl_?impl_->connection_count:0; }
std::size_t Model::property_count() const noexcept { return impl_?impl_->property_count:0; }
std::size_t Model::owned_payload_bytes() const noexcept { return impl_?impl_->owned_bytes:sizeof(Model); }
std::size_t Model::startup_payload_bytes() const noexcept { return impl_?impl_->startup_bytes:sizeof(Model); }
const ConnectionInput* Model::connections() const noexcept { return impl_?impl_->connections.get():nullptr; }
const PropertyInput* Model::properties() const noexcept { return impl_?impl_->properties.get():nullptr; }
const Reference* Model::references() const noexcept { return impl_?impl_->references.get():nullptr; }
const History* Model::initial_histories() const noexcept { return impl_?impl_->initial.get():nullptr; }
const EndpointMass* Model::endpoint_mass() const noexcept { return impl_?impl_->mass.get():nullptr; }
} // namespace tl::fea::type25
