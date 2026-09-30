// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Source.h"
#include "Layout.h"
#include "../activity_operands/State.h"
#include "lib_src/elements/publication/PhysicalActivitySnapshot.h"
namespace tlfea::contact::radioss_type25::runtime_detail {
// Startup-only source relations and the same forecasts used by initialization.
// No CPU incidence copy survives the drained upload into ActivityRuntime.
struct ActivityPlan {
  activity_source::Plan source;
  tl::fea::PhysicalActivityForecast snapshot;
  activity_operands::Forecast operands;
  activity_source::Limits source_limits;
  tl::fea::PhysicalActivityLimits snapshot_limits;
  activity_operands::Limits operand_limits;
  std::size_t host_bytes=0,startup_host_bytes=0,device_bytes=0;
};
TransactionReport PrepareActivity(const TransactionConfig&,const FixedMainSource&,
    const tl::fea::ShellPhysicalBinding&,TransactionLimits,const SourceStaging&,ActivityPlan&) noexcept;
TransactionReport PrepareActivity(const TransactionConfig&,const MovingMainSource&,
    const tl::fea::ShellPhysicalBinding&,TransactionLimits,const SourceStaging&,ActivityPlan&) noexcept;
TransactionReport PrepareActivity(const TransactionConfig&,const MixedMovingMainSource&,
    const tl::fea::ShellPhysicalBinding&,TransactionLimits,const SourceStaging&,ActivityPlan&) noexcept;
activity_operands::BorrowedSlot ActivitySlotShape(const ContactSourceInput&,bool normals,std::size_t free_count) noexcept;
activity_operands::BorrowedSlot ActivitySlot(void*,const Layout&,const ContactSourceInput&,NormalShape) noexcept;
}
