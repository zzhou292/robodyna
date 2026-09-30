// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Internal.h"
namespace tl::fea::solids::control {
struct Selection::Impl {
  std::uint64_t source_instance_id=0;
  UnitScale units{};
  std::uint32_t nvsiz=0,mvsiz=0;
  std::size_t controlled_count=0;
  detail::Layout layout;
  detail::Storage storage;
};
} // namespace tl::fea::solids::control
