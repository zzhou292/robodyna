#pragma once

#include "ReissnerShellBatch.h"

namespace tl::fea::reissner::batch_detail {

struct Model {
  ReissnerShellBatchConfig config;
  ReissnerShellBatchElement element[MaxReissnerShellBatchElements];
  ShellMass element_mass[MaxReissnerShellBatchElements];
  double nodal_mass[MaxTranslationNodes]{};
  double nodal_inertia[MaxTranslationNodes]{};  // Total isotropic J; not J+J_d.
};
struct Control {
  ShellBatchStatus status = ShellBatchStatus::kSuccess;
  ShellStatus element_status = ShellStatus::kSuccess;
  std::uint32_t element = UINT32_MAX, node = UINT32_MAX;
  ShellBatchDiagnostics diagnostics;
};
struct Storage {
  Model model;
  // One attempt's contribution only; never accepted state or total nodal force.
  double force[6 * MaxTranslationNodes]{};
  ShellResult element_result[MaxReissnerShellBatchElements];
  ShellBatchDiagnostics base;
  Control control;
};

}  // namespace tl::fea::reissner::batch_detail
