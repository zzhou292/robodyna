// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Type45Types.h"
#include "../../constraints/NodalRigidAssemblyBinding.h"

namespace tl::fea::type45 {
struct BodyIdentity {
  RigidBindingSourceKind kind = RigidBindingSourceKind::NodalGroup;
  std::uint64_t source_id = 0;
};
struct JointInput {
  Property property;
  GeometryInput geometry;
  BodyIdentity body[2];
};
struct ModelInput {
  std::uint64_t source_instance_id = 0;
  util::ConstView<JointInput> joints{nullptr,0};
};
struct Joint {
  Property property;
  GeometryInput geometry;
  std::size_t domain_nodes[3]{SIZE_MAX,SIZE_MAX,SIZE_MAX};
  std::size_t body_groups[2]{SIZE_MAX,SIZE_MAX};
  // RINI45_RB startup damping values, distinct from the force-stage main-node
  // coefficients required later by automatic stiffness.
  DampingEndpoint damping[2];
};
struct ModelLimits {
  std::size_t max_joints = 4096, max_nodes = 524288;
  std::size_t max_host_bytes = 1024u << 20;
};
enum class ModelStatus {
  Success, AlreadyInitialized, InvalidInput, ResourceLimit, DuplicateIdentity,
  SourceMismatch, UnsupportedProfile
};
struct ModelReport {
  ModelStatus status = ModelStatus::Success;
  const char* message = "OK";
  std::size_t joint = SIZE_MAX, slot = SIZE_MAX;
  explicit operator bool() const noexcept { return status == ModelStatus::Success; }
};

// Closed first physical profile: kinds 1/2/3, two actual rigid members and zero
// optional free K/C. Retains the complete existing domain/rigid authority and
// source-order geometry. It creates no automatic K, force history or clock.
// Source-card/default and complete case coverage remain the app's obligations.
class Model {
 public:
  Model() = default;
  Model(const Model&) noexcept = default;
  Model(Model&& other) noexcept : impl_(other.impl_) {}
  Model& operator=(const Model&) = delete;
  Model& operator=(Model&&) = delete;
  ModelReport Initialize(const NodalRigidAssemblyBinding&, ModelInput, ModelLimits = {}) noexcept;
  bool prepared() const noexcept { return bool(impl_); }
  std::uint64_t source_instance_id() const noexcept;
  const NodalRigidAssemblyBinding* rigid_binding() const noexcept;
  const NodalNodeDomain* domain() const noexcept;
  util::ConstView<Joint> joints() const noexcept;
  bool SharesStorage(const Model&) const noexcept;
  std::size_t owned_payload_bytes() const noexcept;
  std::size_t startup_payload_bytes() const noexcept;
 private:
  struct Impl;
  std::shared_ptr<const Impl> impl_;
};
} // namespace tl::fea::type45
