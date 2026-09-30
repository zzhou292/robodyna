// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../ShellExecutionBinding.h"
#include "../../elements/ShellBatchPlasticityBindingInternal.h"
#include "../../../lib_utils/BoundedArena.h"
#include "../../../lib_utils/SourceIdentityIndex.h"

namespace tl::fea::shell_execution_detail {
using shell_plasticity_binding_detail::Error;
using Status = ShellPlasticityBindingStatus;
using Report = ShellPlasticityBindingReport;
using Index = util::SourceIdentityIndex<0>;
struct Layout {
  util::ArenaRegion parents, qeph, t3, qbat;
  std::size_t arena_bytes = 0;
  ShellExecutionForecast forecast;
};
struct Storage {
  Storage(const ShellBatchPlasticityBinding& c, const NodalRigidAssemblyBinding& r)
      : catalog(c), rigid(r) {}
  ShellBatchPlasticityBinding catalog;
  NodalRigidAssemblyBinding rigid;
  util::HostArena arena;
  ShellExecutionParent* parents = nullptr;
  std::size_t *qeph = nullptr, *t3 = nullptr, *qbat = nullptr;
  ShellExecutionCounts counts;
  ShellExecutionForecast forecast;
};
Report Forecast(const ShellBatchPlasticityBinding&, const NodalCoefficientLedger&,
    const NodalRigidAssemblyBinding&, ShellExecutionLimits, std::size_t header, Layout&) noexcept;
Report Bind(Storage&);
} // namespace tl::fea::shell_execution_detail
