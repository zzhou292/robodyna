#pragma once
#include "ShellBatchPlasticity.h"
#include "ShellSectionLaw.h"
#include "sections/ShellLayeredLaw1.h"
#include <type_traits>

namespace tl::fea {
// Readback value only. Availability follows the explicit immutable source law;
// elastic callers cannot observe manufactured plastic strain/rate/yield fields.
// Active fields are the value contract. Padding/inactive union bytes are not a
// serialization, equality or hash contract; serializers must use typed accessors.
class ShellBatchLayeredSection {
 public:
  ShellBatchLayeredSection() noexcept=default;
  ShellSectionLaw law() const noexcept { return law_; }
  const ShellBatchSectionState* plastic() const noexcept {
    return law_==ShellSectionLaw::LayeredLaw44Nip3?&value_.plastic:nullptr;
  }
  const sections::ShellLayeredLaw1History* elastic() const noexcept {
    return law_==ShellSectionLaw::LayeredLaw1Nip3?&value_.elastic:nullptr;
  }
  static ShellBatchLayeredSection Plastic(const ShellBatchSectionState& value) noexcept {
    return {ShellSectionLaw::LayeredLaw44Nip3,Payload(value)};
  }
  static ShellBatchLayeredSection Elastic(const sections::ShellLayeredLaw1History& value) noexcept {
    return {ShellSectionLaw::LayeredLaw1Nip3,Payload(value)};
  }
 private:
  union Payload {
    ShellBatchSectionState plastic;
    sections::ShellLayeredLaw1History elastic;
    Payload() noexcept:plastic{} {}
    explicit Payload(const ShellBatchSectionState& v) noexcept:plastic(v) {}
    explicit Payload(const sections::ShellLayeredLaw1History& v) noexcept:elastic(v) {}
  };
  ShellBatchLayeredSection(ShellSectionLaw law,Payload value) noexcept:law_(law),value_(value) {}
  ShellSectionLaw law_=ShellSectionLaw::Unspecified;
  Payload value_;
};
static_assert(std::is_trivially_copyable_v<ShellBatchLayeredSection>);
} // namespace tl::fea
