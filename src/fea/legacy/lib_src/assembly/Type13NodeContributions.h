// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "NodalNodeDomain.h"
#include "../elements/type13/Type13Model.h"

namespace tl::fea {
enum class Type13ContributionPhase { PreparedEndpointSI };
enum class Type13ContributionOrder { ModelThenEndpoint };
struct Type13NodeContribution {
  std::size_t model_connection=SIZE_MAX;
  unsigned endpoint=0; // Only N1/N2, represented as 0/1.
  type13::EndpointContribution value;
};
struct Type13ContributionLimits {
  std::size_t max_connections=8192,max_nodes=524288,max_host_bytes=256*1024*1024;
};

// Exact qualified endpoint values associated with a declared immutable domain.
// Records retain model order, not a full native cross-family reduction order.
// Total native J already includes added J; the latter is attribution only.
// N3 contributes nothing, including when another role maps it to this domain.
// No nodal totals, inverse coefficients, DOF resolution or owner is constructed.
class Type13NodeContributions {
 public:
  Type13NodeContributions()=default;
  Type13NodeContributions(const Type13NodeContributions&) noexcept=default;
  Type13NodeContributions(Type13NodeContributions&& other) noexcept
      :Type13NodeContributions(static_cast<const Type13NodeContributions&>(other)) {}
  Type13NodeContributions& operator=(const Type13NodeContributions&)=delete;
  // Report.node identifies a model-local source node, or SIZE_MAX for scope/caps.
  NodalDomainReport Initialize(const type13::Model&,const NodalNodeDomain&,
      Type13ContributionLimits={}) noexcept;
  bool prepared() const noexcept {return bool(impl_);}
  const type13::Model* model() const noexcept;
  const NodalNodeDomain* domain() const noexcept;
  std::size_t record_count() const noexcept;
  tl::util::ConstView<Type13NodeContribution> records() const noexcept;
  bool Matches(const type13::Model&,const NodalNodeDomain&) const noexcept;
  bool Matches(const Type13NodeContributions&) const noexcept;
  std::size_t owned_payload_bytes() const noexcept;
  std::size_t startup_payload_bytes() const noexcept;
  static constexpr Type13ContributionPhase phase() noexcept {return Type13ContributionPhase::PreparedEndpointSI;}
  static constexpr Type13ContributionOrder order() noexcept {return Type13ContributionOrder::ModelThenEndpoint;}
 private:
  struct Impl;
  std::shared_ptr<const Impl> impl_;
};
} // namespace tl::fea
