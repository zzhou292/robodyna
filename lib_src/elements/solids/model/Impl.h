// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Internal.h"
namespace tl::fea::solids {
struct Model::Impl {
  explicit Impl(const SolidNodeContributions& c,ModelProfile p):coefficients(c),profile(p) {}
  SolidNodeContributions coefficients;
  ModelProfile profile;
  control::Selection controls;
  model_detail::Storage storage;
  model_detail::Layout layout;
};
} // namespace tl::fea::solids
