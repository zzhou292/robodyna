// SPDX-License-Identifier: AGPL-3.0-or-later
#include "InitialSeed.h"
#include "Storage.h"
#include <type_traits>
namespace tlfea::contact::radioss_type25::runtime_detail {
namespace {
template<class Source>
TransactionReport Build(const TransactionConfig& config,const Source& source,
    const initial_source::PreparedSource& prepared,const startup::FixedMainView* ready,
    const tl::fea::ShellPhysicalBinding& physical,TransactionLimits limits,std::size_t fixed_bytes,
    Plan& plan,Source& bound,GeneralTransactionForecast& forecast) noexcept {
  const startup::Snapshot* starter=nullptr;
  if constexpr(!std::is_same_v<Source,FixedMainSource>)starter=&source.starter;
  lifecycle::SourceView selection;
  auto report=InitialSeedAccess::Bind(config,source,prepared,physical,starter,ready,selection);
  if(report.status!=TransactionStatus::Ok)return report;
  Source next=source;next.selection=selection;
  report=PreparePlan(config,next,physical,limits,fixed_bytes,plan,InitialSeedAccess::MainRoster(prepared));
  if(report.status!=TransactionStatus::Ok)return report;
  report=InitialSeedAccess::Forecast(prepared,plan.forecast,limits,forecast);
  if(report.status==TransactionStatus::Ok)bound=next;
  return report;
}
}
TransactionReport PrepareGeneralPlan(const TransactionConfig& config,const FixedMainSource& source,
    const startup::FixedMainView& ready,const initial_source::PreparedSource& prepared,const tl::fea::ShellPhysicalBinding& physical,
    TransactionLimits limits,std::size_t fixed_bytes,Plan& plan,FixedMainSource& bound,GeneralTransactionForecast& forecast) noexcept {
  return Build(config,source,prepared,&ready,physical,limits,fixed_bytes,plan,bound,forecast);
}
TransactionReport PrepareGeneralPlan(const TransactionConfig& config,const MovingMainSource& source,
    const initial_source::PreparedSource& prepared,const tl::fea::ShellPhysicalBinding& physical,
    TransactionLimits limits,std::size_t fixed_bytes,Plan& plan,MovingMainSource& bound,GeneralTransactionForecast& forecast) noexcept {
  return Build(config,source,prepared,nullptr,physical,limits,fixed_bytes,plan,bound,forecast);
}
TransactionReport PrepareGeneralPlan(const TransactionConfig& config,const MixedMovingMainSource& source,
    const initial_source::PreparedSource& prepared,const tl::fea::ShellPhysicalBinding& physical,
    TransactionLimits limits,std::size_t fixed_bytes,Plan& plan,MixedMovingMainSource& bound,GeneralTransactionForecast& forecast) noexcept {
  return Build(config,source,prepared,nullptr,physical,limits,fixed_bytes,plan,bound,forecast);
}
}
namespace tlfea::contact::radioss_type25 {
TransactionReport Transaction::GeneralPreflight(const TransactionConfig& config,const FixedMainSource& source,
    const startup::FixedMainView& ready,const initial_source::PreparedSource& prepared,const tl::fea::ShellPhysicalBinding& physical,
    GeneralTransactionForecast& output,TransactionLimits limits) noexcept {
  runtime_detail::Plan plan(limits);FixedMainSource bound;GeneralTransactionForecast forecast;
  const auto report=runtime_detail::PrepareGeneralPlan(config,source,ready,prepared,physical,limits,sizeof(Transaction)+sizeof(Impl),plan,bound,forecast);
  if(report.status==TransactionStatus::Ok)output=forecast;return report;
}
TransactionReport Transaction::GeneralPreflight(const TransactionConfig& config,const MovingMainSource& source,
    const initial_source::PreparedSource& prepared,const tl::fea::ShellPhysicalBinding& physical,
    GeneralTransactionForecast& output,TransactionLimits limits) noexcept {
  runtime_detail::Plan plan(limits);MovingMainSource bound;GeneralTransactionForecast forecast;
  const auto report=runtime_detail::PrepareGeneralPlan(config,source,prepared,physical,limits,sizeof(Transaction)+sizeof(Impl),plan,bound,forecast);
  if(report.status==TransactionStatus::Ok)output=forecast;return report;
}
TransactionReport Transaction::GeneralPreflight(const TransactionConfig& config,const MixedMovingMainSource& source,
    const initial_source::PreparedSource& prepared,const tl::fea::ShellPhysicalBinding& physical,
    GeneralTransactionForecast& output,TransactionLimits limits) noexcept {
  runtime_detail::Plan plan(limits);MixedMovingMainSource bound;GeneralTransactionForecast forecast;
  const auto report=runtime_detail::PrepareGeneralPlan(config,source,prepared,physical,limits,sizeof(Transaction)+sizeof(Impl),plan,bound,forecast);
  if(report.status==TransactionStatus::Ok)output=forecast;return report;
}
}
