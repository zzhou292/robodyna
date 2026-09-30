// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Type13ModelInternal.h"
#include "Type13Startup.h"
#include <algorithm>

namespace tl::fea::type13 {
using namespace model_detail;
struct Model::Impl {
  std::uint64_t source_instance_id=0;
  WorkingUnits units{};
  std::size_t global_nodes=0;
  Layout layout;
  util::HostArena arena;
  ModelNode* nodes=nullptr;
  OwnedProperty* properties=nullptr;
  ModelConnection* connections=nullptr;
  Startup* startup=nullptr;
};
ModelReport Model::Initialize(const ModelInput& input,ModelLimits limits) noexcept {
  if(impl_)return Error(ModelStatus::AlreadyInitialized,"TYPE13 model is immutable");
  Layout layout;
  auto report=Preflight(input,limits,sizeof(Model)+sizeof(Impl)+256,layout);
  if(!report)return report;
  try {
    auto draft=std::make_shared<Impl>();
    draft->source_instance_id=input.source_instance_id;draft->units=input.units;
    draft->global_nodes=input.global_node_count;draft->layout=layout;
    if(!draft->arena.Initialize(layout.arena_bytes))return Error(ModelStatus::ResourceLimit,"TYPE13 arena allocation failed");
    draft->nodes=draft->arena.Construct<ModelNode>(layout.nodes);
    draft->properties=draft->arena.Construct<OwnedProperty>(layout.properties);
    draft->connections=draft->arena.Construct<ModelConnection>(layout.connections);
    draft->startup=draft->arena.Construct<Startup>(layout.startup);
    if(!draft->nodes||!draft->properties||!draft->connections||!draft->startup)
      return Error(ModelStatus::ResourceLimit,"TYPE13 arena layout is invalid");
    Scratch scratch;
    report=CheckNodes(input,scratch);if(!report)return report;
    std::copy_n(input.nodes,input.node_count,draft->nodes);
    std::copy_n(input.connections,input.connection_count,draft->connections);
    for(std::size_t p=0;p<input.property_count;++p) {
      if(scratch.property_ids.First(input.properties[p].source_id)!=p)
        return Error(ModelStatus::DuplicateIdentity,"Repeated TYPE13 source property",ModelEntry::Property,p);
      auto& owned=draft->properties[p];owned.declaration=input.properties[p];
      for(unsigned c=0;c<CurveCount;++c) {
        std::copy_n(input.properties[p].input.curves[c].points,CurvePoints,owned.curves[c].points);
        owned.declaration.input.curves[c]={owned.curves[c].points,CurvePoints};
      }
      const auto status=InitializeProperty(owned.declaration.input,owned.value);
      if(status!=Status::Success)
        return Error(ModelStatus::NumericalFailure,"TYPE13 property preparation rejected",ModelEntry::Property,p,status);
    }
    for(std::size_t e=0;e<input.connection_count;++e) {
      report=CheckConnection(input,e,scratch);if(!report)return report;
      const auto& connection=input.connections[e];
      const auto status=InitializeElement(draft->properties[connection.property].value,
          ReferenceFor(input,connection),draft->startup[e]);
      if(status!=Status::Success)
        return Error(ModelStatus::NumericalFailure,"TYPE13 reference preparation rejected",ModelEntry::Connection,e,status);
      scratch.used_properties[connection.property]=true;
      for(auto node:connection.node)scratch.used_nodes[node]=true;
    }
    for(std::size_t n=0;n<input.node_count;++n)if(!scratch.used_nodes[n])
      return Error(ModelStatus::InvalidInput,"Unreferenced TYPE13 source node",ModelEntry::Node,n);
    for(std::size_t p=0;p<input.property_count;++p)if(!scratch.used_properties[p])
      return Error(ModelStatus::InvalidInput,"Unreferenced TYPE13 source property",ModelEntry::Property,p);
    impl_=std::move(draft);return {};
  } catch(const std::bad_alloc&) {
    return Error(ModelStatus::ResourceLimit,"TYPE13 startup allocation failed");
  }
}
bool Model::prepared() const noexcept { return bool(impl_); }
bool Model::SharesStorage(const Model& other) const noexcept { return impl_&&impl_==other.impl_; }
bool Model::Matches(const Model& other) const noexcept {
  if(!impl_||!other.impl_)return false;
  if(SharesStorage(other))return true;
  if(source_instance_id()!=other.source_instance_id()||!Same(units(),other.units())||
     global_node_count()!=other.global_node_count()||node_count()!=other.node_count()||
     property_count()!=other.property_count()||connection_count()!=other.connection_count())return false;
  for(std::size_t n=0;n<node_count();++n) {
    const auto& a=nodes()[n];const auto& b=other.nodes()[n];
    if(a.source_id!=b.source_id||a.global_node!=b.global_node||!Same(a.position_native,b.position_native))return false;
  }
  for(std::size_t p=0;p<property_count();++p)
    if(!Same(*property_declaration(p),*other.property_declaration(p)))return false;
  for(std::size_t e=0;e<connection_count();++e)
    if(!Same(connections()[e],other.connections()[e]))return false;
  return true;
}
std::uint64_t Model::source_instance_id() const noexcept { return impl_?impl_->source_instance_id:0; }
WorkingUnits Model::units() const noexcept { return impl_?impl_->units:WorkingUnits{}; }
std::size_t Model::node_count() const noexcept { return impl_?impl_->layout.nodes.count:0; }
std::size_t Model::global_node_count() const noexcept { return impl_?impl_->global_nodes:0; }
std::size_t Model::property_count() const noexcept { return impl_?impl_->layout.properties.count:0; }
std::size_t Model::connection_count() const noexcept { return impl_?impl_->layout.connections.count:0; }
std::size_t Model::owned_payload_bytes() const noexcept { return impl_?impl_->layout.owned_bytes:sizeof(Model); }
std::size_t Model::startup_payload_bytes() const noexcept { return impl_?impl_->layout.startup_bytes:sizeof(Model); }
const ModelNode* Model::nodes() const noexcept { return impl_?impl_->nodes:nullptr; }
const ModelConnection* Model::connections() const noexcept { return impl_?impl_->connections:nullptr; }
const ModelPropertyInput* Model::property_declaration(std::size_t p) const noexcept {
  return p<property_count()?&impl_->properties[p].declaration:nullptr;
}
const Property* Model::property(std::size_t p) const noexcept { return p<property_count()?&impl_->properties[p].value:nullptr; }
const Startup* Model::startup(std::size_t e) const noexcept { return e<connection_count()?&impl_->startup[e]:nullptr; }
bool Model::Endpoint(std::size_t e,unsigned local,EndpointContribution& out) const noexcept {
  if(e>=connection_count()||local>=2)return false;
  const auto& c=impl_->connections[e];const auto& n=impl_->nodes[c.node[local]];
  out={c.source_id,impl_->properties[c.property].declaration.source_id,n.source_id,n.global_node,
       impl_->startup[e].endpoint};
  return true;
}
} // namespace tl::fea::type13
