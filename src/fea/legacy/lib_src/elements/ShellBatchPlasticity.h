#pragma once
#include "sections/ShellLayeredJ2.h"
#include <cstdint>

namespace tl::fea {
// Opt-in tabulated plastic material with explicit optional source rate settings.
// Existing batch configuration
// records and LAW1 device storage remain unchanged. Initialize copies all data;
// the caller may release this declaration and its curve storage afterwards.
// Joined collections with per-parent materials use ShellBatchPlasticityBinding
// instead; the original single-material API and scope rules remain available.
struct ShellBatchPlasticityConfig {
  std::uint64_t material_id=0,curve_id=0;
  material::TabulatedShellPlasticityCurve curve{};
  material::TabulatedShellPlasticityRate rate{};
};
struct ShellBatchSectionState {
  sections::ShellLayeredJ2History history{};
  sections::ShellLayeredJ2Diagnostics diagnostics{};
  // Native point plastic-work diagnostic integrated with current area and
  // interval force thickness. Never add this again to the total shell work ledger.
  double cumulative_plastic_work_J=0;
};
} // namespace tl::fea
