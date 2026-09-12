// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../elements/beam18/Model.h"
namespace tl::fea {
enum class Beam18ContributionPhase { PreparedEndpointSI };
enum class Beam18ContributionOrder { ModelThenEndpoint };
struct Beam18NodeContribution {
  std::size_t model_parent=SIZE_MAX;
  unsigned endpoint=0;
  beam18::EndpointContribution value;
};
struct Beam18ContributionLimits {
  std::size_t max_parents=1024,max_nodes=524288,max_host_bytes=256*1024*1024;
};
// Exact PMASS endpoint M and total scalar J; no invented inertia partition.
// Two source-order records per beam. Orientation N3 adds no coefficient.
class Beam18NodeContributions {
 public:
  Beam18NodeContributions()=default;
  Beam18NodeContributions(const Beam18NodeContributions&) noexcept=default;
  Beam18NodeContributions(Beam18NodeContributions&& other) noexcept
      :Beam18NodeContributions(static_cast<const Beam18NodeContributions&>(other)) {}
  Beam18NodeContributions& operator=(const Beam18NodeContributions&)=delete;
  NodalDomainReport Initialize(const beam18::Model&,Beam18ContributionLimits={}) noexcept;
  bool prepared() const noexcept {return bool(impl_);}
  const beam18::Model* model() const noexcept;
  const NodalNodeDomain* domain() const noexcept;
  std::size_t record_count() const noexcept;
  util::ConstView<Beam18NodeContribution> records() const noexcept;
  bool Matches(const beam18::Model&,const NodalNodeDomain&) const noexcept;
  bool Matches(const Beam18NodeContributions&) const noexcept;
  std::size_t owned_payload_bytes() const noexcept;
  std::size_t startup_payload_bytes() const noexcept;
  static constexpr Beam18ContributionPhase phase() noexcept {return Beam18ContributionPhase::PreparedEndpointSI;}
  static constexpr Beam18ContributionOrder order() noexcept {return Beam18ContributionOrder::ModelThenEndpoint;}
 private:
  struct Impl;
  std::shared_ptr<const Impl> impl_;
};
} // namespace tl::fea
