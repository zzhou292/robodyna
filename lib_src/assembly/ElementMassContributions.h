// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "NodalNodeDomain.h"

namespace tl::fea {
struct ElementMassSource {
  std::uint64_t source_element_id=0,source_node_id=0;
  std::size_t domain_node=SIZE_MAX;
  double mass_source=0;
};
struct ElementMassInput {
  std::uint64_t source_instance_id=0;
  double mass_to_kg=0;
  const ElementMassSource* records=nullptr;
  std::size_t record_count=0;
};
struct ElementMassContribution {
  ElementMassSource source{};
  double mass_kg=0;
  // ADMAS TYPE5 does not write scalar nodal inertia.
  static constexpr double isotropic_inertia_kg_m2() noexcept {return 0;}
};
struct ElementMassLimits {
  std::size_t max_records=4096,max_nodes=524288,max_host_bytes=128*1024*1024;
};
enum class ElementMassPolicy { NonnegativeAdditiveType5 };
enum class ElementMassPhase { SourceMassConvertedToSI };

// Closed additive ELEMENT_MASS / ADMAS5 producer, in supplied source-row order.
// A card has no coordinates: the complete immutable domain authenticates NID,
// index and coordinates. Source-file/card coverage remains the app's obligation.
// Distinct EIDs may target one node. Zero mass is retained as a source occurrence;
// it does not establish a positive DOF. No part distribution/replacement forms.
class ElementMassContributions {
 public:
  ElementMassContributions()=default;
  ElementMassContributions(const ElementMassContributions&) noexcept=default;
  ElementMassContributions(ElementMassContributions&& other) noexcept
      :ElementMassContributions(static_cast<const ElementMassContributions&>(other)) {}
  ElementMassContributions& operator=(const ElementMassContributions&)=delete;
  // Report.node is the input record index for row errors, otherwise SIZE_MAX.
  NodalDomainReport Initialize(const NodalNodeDomain&,ElementMassInput,
                              ElementMassLimits={}) noexcept;
  bool prepared() const noexcept {return bool(impl_);}
  const NodalNodeDomain* domain() const noexcept;
  double mass_to_kg() const noexcept;
  tl::util::ConstView<ElementMassContribution> records() const noexcept;
  bool Matches(const ElementMassContributions&) const noexcept;
  std::size_t owned_payload_bytes() const noexcept;
  std::size_t startup_payload_bytes() const noexcept;
  static constexpr ElementMassPolicy policy() noexcept {
    return ElementMassPolicy::NonnegativeAdditiveType5;
  }
  static constexpr ElementMassPhase phase() noexcept {
    return ElementMassPhase::SourceMassConvertedToSI;
  }
 private:
  struct Impl;
  std::shared_ptr<const Impl> impl_;
};
} // namespace tl::fea
