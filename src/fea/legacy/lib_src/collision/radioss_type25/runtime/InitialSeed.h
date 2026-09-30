// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Planning.h"
#include "../../RadiossType25InitialState.h"
#include "../../RadiossType25Transaction.h"
namespace tlfea::contact::radioss_type25::runtime_detail {
// Sole production reader of the opaque seed. It never imports a caller row
// array and cannot create physical/publication authority.
struct InitialSeedAccess {
  static InitialMainRoster MainRoster(const initial_source::PreparedSource&) noexcept;
  static TransactionReport Bind(const TransactionConfig&,const ContactSourceInput&,
      const initial_source::PreparedSource&,const tl::fea::ShellPhysicalBinding&,
      const startup::Snapshot*,const startup::FixedMainView*,lifecycle::SourceView&) noexcept;
  static TransactionReport Forecast(const initial_source::PreparedSource&,const TransactionForecast&,
      TransactionLimits,GeneralTransactionForecast&) noexcept;
  static TransactionReport Upload(const initial_source::PreparedSource&,Device,cudaStream_t,TransactionInitializationDiagnostics&) noexcept;
};
TransactionReport PrepareGeneralPlan(const TransactionConfig&,const FixedMainSource&,const startup::FixedMainView&,
    const initial_source::PreparedSource&,const tl::fea::ShellPhysicalBinding&,TransactionLimits,std::size_t,
    Plan&,FixedMainSource&,GeneralTransactionForecast&) noexcept;
TransactionReport PrepareGeneralPlan(const TransactionConfig&,const MovingMainSource&,
    const initial_source::PreparedSource&,const tl::fea::ShellPhysicalBinding&,TransactionLimits,std::size_t,
    Plan&,MovingMainSource&,GeneralTransactionForecast&) noexcept;
TransactionReport PrepareGeneralPlan(const TransactionConfig&,const MixedMovingMainSource&,
    const initial_source::PreparedSource&,const tl::fea::ShellPhysicalBinding&,TransactionLimits,std::size_t,
    Plan&,MixedMovingMainSource&,GeneralTransactionForecast&) noexcept;
}
