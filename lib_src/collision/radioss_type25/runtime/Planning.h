// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Source.h"
#include "Layout.h"
#include "lib_src/collision/RadiossType25AssemblyDevice.h"
namespace tlfea::contact::radioss_type25::runtime_detail {
// Owning source staging and exact allocation plan shared by the public forecast
// and actual initializer. It grants no source/publication/clock authority.
struct Plan {
  explicit Plan(TransactionLimits limits):readback(limits.max_host_bytes){}
  Plan(const Plan&)=delete;Plan& operator=(const Plan&)=delete;
  SourceStaging upload;
  TransactionForecast forecast;
  Layout layout;
  NormalShape normal;
  assembly::IncidenceLimits incidence_limits;
  tl::util::BoundedArenaLayout readback;
  tl::util::ArenaRegion rows,secondary;
};
TransactionReport PreparePlan(const TransactionConfig&,const FixedMainSource&,
    const tl::fea::ShellPhysicalBinding&,TransactionLimits,std::size_t fixed_host_bytes,Plan&) noexcept;
TransactionReport PreparePlan(const TransactionConfig&,const MovingMainSource&,
    const tl::fea::ShellPhysicalBinding&,TransactionLimits,std::size_t fixed_host_bytes,Plan&) noexcept;
TransactionReport PreparePlan(const TransactionConfig&,const MixedMovingMainSource&,
    const tl::fea::ShellPhysicalBinding&,TransactionLimits,std::size_t fixed_host_bytes,Plan&) noexcept;
}
