// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "lib_src/elements/ShellBatchBinding.h"
#include <vector>

namespace qbat_binding_test {
// Source fixture owned/authenticated by qualification/qbat, never duplicated.
struct OriginalCollection {
  std::vector<tl::fea::ShellQbatBindingInput> quads;
  tl::fea::ShellT3BindingInput triangle;
  OriginalCollection();
  tl::fea::ShellFormulationCollectionInput Input() const;
  static constexpr std::size_t NodeCount=4384,QuadCount=4250;
};
} // namespace qbat_binding_test
