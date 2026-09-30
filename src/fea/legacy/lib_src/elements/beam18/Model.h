// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "ForceTypes.h"
#include "../../assembly/NodalNodeDomain.h"

namespace tl::fea::beam18 {
enum class ModelProfile { Unspecified, CircularFourPointLaw44V1 };
struct ParentInput { Reference reference; Material material; };
struct Parent {
  Reference reference;
  std::size_t material_index=SIZE_MAX;
  std::size_t domain_nodes[2]{SIZE_MAX,SIZE_MAX};
};
struct MaterialRecord { std::uint64_t source_material_id=0; Material value; };
struct ModelInput {
  std::uint64_t source_instance_id=0;
  util::ConstView<ParentInput> parents{nullptr,0};
  ModelProfile profile=ModelProfile::Unspecified;
};
struct ModelLimits {
  std::size_t max_parents=1024,max_materials=256,max_curve_points=65536;
  std::size_t max_nodes=524288,max_host_bytes=256*1024*1024;
};
enum class ModelStatus { Success,AlreadyInitialized,InvalidInput,ResourceLimit,
  DuplicateIdentity,IdentityMismatch,PositionMismatch };
struct ModelReport {
  ModelStatus status=ModelStatus::Success;
  const char* message="OK";
  std::size_t parent=SIZE_MAX,local=SIZE_MAX;
  explicit operator bool() const noexcept {return status==ModelStatus::Success;}
};
struct EndpointContribution {
  std::uint64_t source_element_id=0,source_part_id=0,source_section_id=0,source_material_id=0;
  std::uint64_t source_node_id=0;
  std::size_t global_node=SIZE_MAX;
  Endpoint coefficients;
};
// Immutable source-order references and MID-pooled material curves. The exact
// canonical domain is retained. N3 is orientation provenance, never an endpoint.
// This handle owns curves but no force/history cache, DOF authority or clock.
class Model {
 public:
  Model()=default;
  Model(const Model&) noexcept=default;
  Model(Model&& other) noexcept:Model(static_cast<const Model&>(other)) {}
  Model& operator=(const Model&)=delete;
  ModelReport Initialize(const NodalNodeDomain&,ModelInput,ModelLimits={}) noexcept;
  bool prepared() const noexcept {return bool(impl_);}
  util::ConstView<Parent> parents() const noexcept;
  util::ConstView<MaterialRecord> materials() const noexcept;
  const NodalNodeDomain* domain() const noexcept;
  ModelProfile profile() const noexcept;
  std::uint64_t source_instance_id() const noexcept;
  bool Endpoint(std::size_t parent,unsigned local,EndpointContribution&) const noexcept;
  bool Matches(const Model&) const noexcept;
  bool SharesStorage(const Model&) const noexcept;
  std::size_t owned_payload_bytes() const noexcept;
  std::size_t startup_payload_bytes() const noexcept;
 private:
  struct Impl;
  std::shared_ptr<const Impl> impl_;
};
} // namespace tl::fea::beam18
