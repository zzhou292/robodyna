// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "lib_src/elements/solid18/Solid18Types.h"

namespace tl::fea::solid18::law44 {
using ReferenceInput = solid18::ReferenceInput;
// This is the explicit selected native profile, not a topology-based law map.
TL_SOLID18_HD inline ResolvedProfile Profile() noexcept {
  return {44, 18, 17, 2, 2, 2, 2, 1, 2, 1};
}
enum class SourceTopology : std::uint8_t { EightDistinct, RepeatedPairs56And78 };

class Reference {
 public:
  TL_SOLID18_HD bool prepared() const noexcept { return prepared_; }
  TL_SOLID18_HD const ReferenceInput& input() const noexcept { return input_; }
  TL_SOLID18_HD const StartupGeometry& geometry() const noexcept { return geometry_; }
  TL_SOLID18_HD const Mass& mass() const noexcept { return mass_; }
  TL_SOLID18_HD SourceTopology topology() const noexcept { return topology_; }
  TL_SOLID18_HD unsigned source_slot(unsigned native_slot) const noexcept {
    return native_slot < 8 ? native_to_source_[native_slot] : 8;
  }
 private:
  ReferenceInput input_{};
  StartupGeometry geometry_{};
  Mass mass_{};
  std::uint8_t native_to_source_[8]{};
  SourceTopology topology_ = SourceTopology::EightDistinct;
  bool prepared_ = false;
  friend TL_SOLID18_HD Status InitializeReference(const ReferenceInput&, Reference&) noexcept;
};
}  // namespace tl::fea::solid18::law44
