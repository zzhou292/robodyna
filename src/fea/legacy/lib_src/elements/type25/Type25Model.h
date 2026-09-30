// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Type25Types.h"
#include <memory>

namespace tl::fea::type25 {
struct ModelReport {
  Status status=Status::InvalidInput;
  const char* message="Invalid TYPE25 startup input";
  std::size_t connection=SIZE_MAX;
  explicit operator bool() const noexcept { return status==Status::Success; }
};

// Immutable source-independent startup ownership only. Copies complete input
// ranges after count/byte preflight. No nodal state, mass accumulation, solver,
// source parser, clock or CUDA allocation. Failures leave this model untouched.
// Repeated endpoints across distinct springs are allowed and remain separate
// mass/J contributions; each repeated node identity/position must agree exactly.
class Model {
 public:
  Model();
  ~Model();
  Model(const Model&) noexcept=default;
  Model(Model&& other) noexcept : impl_(other.impl_) {}
  Model& operator=(const Model&)=delete;
  Model& operator=(Model&&)=delete;
  ModelReport Initialize(const ModelInput&) noexcept;
  bool prepared() const noexcept;
  bool Matches(const Model&) const noexcept;
  // Storage identity only, for once-only retained-payload accounting. Equal
  // independently constructed models do not share storage.
  bool SharesStorage(const Model&) const noexcept;
  std::uint64_t source_instance_id() const noexcept;
  SourceUnits source_units() const noexcept;
  std::size_t global_node_count() const noexcept;
  std::size_t connection_count() const noexcept;
  std::size_t property_count() const noexcept;
  std::size_t owned_payload_bytes() const noexcept;
  std::size_t startup_payload_bytes() const noexcept;
  const ConnectionInput* connections() const noexcept;
  const PropertyInput* properties() const noexcept;
  const Reference* references() const noexcept;
  const History* initial_histories() const noexcept;
  const EndpointMass* endpoint_mass() const noexcept; // Exactly 2*connection_count.
 private:
  struct Impl;
  std::shared_ptr<const Impl> impl_;
};
} // namespace tl::fea::type25
