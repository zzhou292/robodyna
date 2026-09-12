// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../QephBatch.h"
#include "../QephForceData.h"
#include "../../ShellBatchLayeredSection.h"
#include "../../ShellBatchFailure.h"
#include "../../ShellBatchPlasticityBinding.h"
#include <cstdint>

namespace tl::fea::qeph::mapped {
inline constexpr std::uint32_t NoActivityFailure = UINT32_MAX;

// The original complete force-validation phase must succeed before this
// parent-ordered agreement phase inspects the freshly read typed histories.
template<class Activity, class FailureActivity>
BatchReport ValidateSectionActivity(const ShellBatchPlasticityBinding& catalog,
    const ShellBatchLayeredSection* sections, std::size_t count,
    Activity active, FailureActivity failure_active) noexcept {
  if (!sections) {
    return {BatchStatus::InvalidInput,"Mapped Qeph typed section shape is missing"};
  }
  for (std::size_t parent = 0; parent < count; ++parent) {
    ShellSectionLaw law = ShellSectionLaw::Unspecified;
    if (!catalog.Law(ShellBindingFamily::Qeph,parent,&law) || sections[parent].law() != law) {
      return {BatchStatus::NonfiniteResult,"Mapped Qeph typed section role differs",
          static_cast<std::uint32_t>(parent)};
    }
    const bool expected = sections[parent].one_point()
        ? sections[parent].one_point()->point.failure.history.point_active
        : failure_active(parent) != 0;
    if (active(parent) != (expected ? 1 : 0)) {
      return {BatchStatus::NonfiniteResult,"Mapped Qeph force and failure activity differ",
          static_cast<std::uint32_t>(parent)};
    }
  }
  return {BatchStatus::Success,"OK"};
}

template<class Activity>
BatchReport ValidateSectionActivity(const ShellBatchPlasticityBinding& catalog,
    const ShellBatchLayeredSection* sections, const ShellBatchFailureState* failure,
    std::size_t count, Activity active) noexcept {
  if (!failure) return {BatchStatus::InvalidInput,"Mapped Qeph typed section shape is missing"};
  return ValidateSectionActivity(catalog,sections,count,active,
      [failure](std::size_t parent) { return failure[parent].active ? 1 : 0; });
}

// Compact role bytes describe the actual freshly validated selected section.
// The full typed-history overload above retains its original contract.
template<class Activity, class FailureActivity>
BatchReport ValidateRoleActivity(const ShellBatchPlasticityBinding& catalog,
    const std::uint8_t* roles, std::size_t count, Activity active, FailureActivity failure_active) noexcept {
  if (!roles) return {BatchStatus::InvalidInput,"Mapped Qeph typed section shape is missing"};
  for (std::size_t parent = 0; parent < count; ++parent) {
    ShellSectionLaw law = ShellSectionLaw::Unspecified;
    if (!catalog.Law(ShellBindingFamily::Qeph,parent,&law) ||
        roles[parent] != static_cast<std::uint8_t>(law)) {
      return {BatchStatus::NonfiniteResult,"Mapped Qeph typed section role differs",
          static_cast<std::uint32_t>(parent)};
    }
    if (active(parent) != (failure_active(parent) ? 1 : 0)) {
      return {BatchStatus::NonfiniteResult,"Mapped Qeph force and failure activity differ",
          static_cast<std::uint32_t>(parent)};
    }
  }
  return {BatchStatus::Success,"OK"};
}

template<class Activity>
BatchReport FinishActivity(std::uint32_t first_invalid,
    const ShellBatchPlasticityBinding& catalog, const ShellBatchLayeredSection* sections,
    const ShellBatchFailureState* failure, std::size_t count, Activity active) noexcept {
  if (first_invalid != NoActivityFailure) {
    return {BatchStatus::NonfiniteResult,"Mapped Qeph force cache differs from its source/endpoint role",
        first_invalid};
  }
  return ValidateSectionActivity(catalog,sections,failure,count,active);
}
} // namespace tl::fea::qeph::mapped
