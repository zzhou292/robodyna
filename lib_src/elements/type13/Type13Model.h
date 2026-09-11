// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Type13Types.h"
#include <cstdint>
#include <memory>

namespace tl::fea::type13 {
struct ModelNode {
  std::uint64_t source_id=0;
  // A reference-only N3 may have no mechanical owner index.
  std::size_t global_node=SIZE_MAX;
  Vec3 position_native{};
};
struct ModelPropertyInput { std::uint64_t source_id=0; PropertyInput input{}; };
struct ModelConnection {
  std::uint64_t source_id=0;
  std::size_t property=SIZE_MAX,node[3]{SIZE_MAX,SIZE_MAX,SIZE_MAX};
  Vec3 skew_x{1,0,0},skew_y{0,1,0};
  double coordinate_noise=0;
  unsigned endpoint_release[4]{};
};
struct ModelLimits {
  std::size_t max_connections=8192,max_properties=1024,max_local_nodes=16384;
  std::size_t max_global_nodes=1048576,max_host_bytes=128*1024*1024;
};
struct ModelInput {
  std::uint64_t source_instance_id=0;
  WorkingUnits units{};
  const ModelNode* nodes=nullptr;
  const ModelPropertyInput* properties=nullptr;
  const ModelConnection* connections=nullptr;
  std::size_t node_count=0,property_count=0,connection_count=0,global_node_count=0;
};
enum class ModelStatus { Success,AlreadyInitialized,InvalidInput,ResourceLimit,DuplicateIdentity,NumericalFailure };
enum class ModelEntry { None,Node,Property,Connection };
struct ModelReport {
  ModelStatus status=ModelStatus::Success;
  ModelEntry kind=ModelEntry::None;
  std::size_t entry=SIZE_MAX;
  Status native_status=Status::Success;
  const char* message="";
  explicit operator bool() const noexcept { return status==ModelStatus::Success; }
};
struct EndpointContribution {
  std::uint64_t source_element_id=0,source_property_id=0,source_node_id=0;
  std::size_t global_node=SIZE_MAX;
  // Native total J already contains added J. These are not two additive totals.
  EndpointCoefficients coefficients{};
};

// Immutable startup ownership over the already qualified TYPE13 value functions.
// All source nodes/properties/connections and curve values are owned. N3 carries
// geometry identity but receives no endpoint mass. No force history, clock,
// nodal accumulation or runtime/attachment admission is constructed here.
class Model {
 public:
  Model() noexcept=default;
  Model(const Model&) noexcept=default;
  Model(Model&& other) noexcept:impl_(other.impl_) {}
  Model& operator=(const Model&)=delete;
  Model& operator=(Model&&)=delete;
  ModelReport Initialize(const ModelInput&,ModelLimits={}) noexcept;
  bool prepared() const noexcept;
  bool Matches(const Model&) const noexcept;
  bool SharesStorage(const Model&) const noexcept;
  std::uint64_t source_instance_id() const noexcept;
  WorkingUnits units() const noexcept;
  std::size_t node_count() const noexcept;
  std::size_t global_node_count() const noexcept;
  std::size_t property_count() const noexcept;
  std::size_t connection_count() const noexcept;
  std::size_t owned_payload_bytes() const noexcept;
  std::size_t startup_payload_bytes() const noexcept;
  const ModelNode* nodes() const noexcept;
  const ModelConnection* connections() const noexcept;
  const ModelPropertyInput* property_declaration(std::size_t) const noexcept;
  const Property* property(std::size_t) const noexcept;
  const Startup* startup(std::size_t) const noexcept;
  bool Endpoint(std::size_t connection,unsigned endpoint,EndpointContribution&) const noexcept;
 private:
  struct Impl;
  std::shared_ptr<const Impl> impl_;
};
} // namespace tl::fea::type13
