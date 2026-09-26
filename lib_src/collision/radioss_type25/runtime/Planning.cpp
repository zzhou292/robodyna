// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Planning.h"
#include "Storage.h"
#include <new>
namespace tlfea::contact::radioss_type25::runtime_detail {
namespace fe=tl::fea;
namespace {
TransactionReport Error(TransactionStatus s,const char* message){return {s,message};}
bool Add(std::size_t value,std::size_t& sum){if(value>SIZE_MAX-sum)return false;sum+=value;return true;}
template<class Source>
TransactionReport Build(const TransactionConfig& config,const Source& source,
    const fe::ShellPhysicalBinding& physical,TransactionLimits limits,std::size_t fixed_host_bytes,Plan& plan) noexcept {
  const auto prepared=PrepareSource(config,source,physical,limits,plan.upload);
  if(prepared.status!=TransactionStatus::Ok)return prepared;
  const auto& upload=plan.upload;
  candidates::Forecast inventory;search::Forecast maintenance;assembly::IncidenceForecast incidence;
  const auto candidates_status=candidates::Inventory::Preflight(upload.inventory,limits.inventory,inventory);
  if(candidates_status!=candidates::Status::Ok)return Error(candidates_status==candidates::Status::DeviceFailure?
      TransactionStatus::DeviceFailure:TransactionStatus::ResourceLimit,"Native candidate preflight rejected");
  if(search::Maintenance::Preflight(upload.maintenance,limits.maintenance,maintenance)!=search::Status::Ok)
    return Error(TransactionStatus::ResourceLimit,"Native maintenance preflight rejected");
  plan.incidence_limits={limits.optimized_candidates,source.selection.node_count,
    limits.optimized_candidates/source.force_packet_size+(limits.optimized_candidates%source.force_packet_size!=0),limits.max_device_bytes};
  if(assembly::DeviceIncidenceBuilder::Preflight(plan.incidence_limits,incidence)!=assembly::IncidenceStatus::Ok)
    return Error(TransactionStatus::ResourceLimit,"Native ASS0 incidence preflight rejected");
  std::size_t cub=0;
  if(QueryScratch(source.selection.secondary_count,limits.optimized_candidates,cub)!=cudaSuccess)
    return Error(TransactionStatus::DeviceFailure,"Native runtime scratch query failed");
  plan.normal={upload.moving.enabled,upload.moving.free_main_ids.size(),upload.moving.activation,upload.moving.mixed};
  auto& layout=plan.layout;if(!MakeLayout(source,limits,cub,layout,plan.normal,config.response_mass))return Error(TransactionStatus::ResourceLimit,"Native runtime arena exceeds cap");
  auto& forecast=plan.forecast;forecast.raw_pair_capacity=limits.inventory.max_pairs;
  forecast.optimized_capacity=limits.optimized_candidates;forecast.sliding_capacity=limits.sliding_entries;
  forecast.runtime_device_bytes=layout.bytes;forecast.normal_device_bytes=layout.normal.bytes;
  if(!Add(inventory.device_bytes,forecast.inventory_device_bytes)||!Add(inventory.device_bytes,forecast.inventory_device_bytes)||
     !Add(maintenance.device_bytes,forecast.maintenance_device_bytes)||!Add(maintenance.device_bytes,forecast.maintenance_device_bytes))
    return Error(TransactionStatus::ResourceLimit,"Native paired arena forecast overflow");
  forecast.incidence_device_bytes=incidence.device_bytes;
  if(!Add(layout.bytes,forecast.device_bytes)||!Add(forecast.inventory_device_bytes,forecast.device_bytes)||
     !Add(forecast.maintenance_device_bytes,forecast.device_bytes)||!Add(incidence.device_bytes,forecast.device_bytes)||
     forecast.device_bytes>limits.max_device_bytes)
    return Error(TransactionStatus::ResourceLimit,"Complete native device forecast exceeds cap");
  auto& readback_layout=plan.readback;auto& rows=plan.rows;auto& secondary=plan.secondary;
  if(!readback_layout.Append<NativeGeometryHistory>(source.selection.secondary_count,rows)||
     !readback_layout.Append<lifecycle::Secondary>(source.selection.secondary_count,secondary))
    return Error(TransactionStatus::ResourceLimit,"Native readback forecast exceeds cap");
  fe::ActivePrefixForecast activity;
  if(config.activity==ContactActivityPolicy::AllActivePrefix) {
    const auto checked=fe::PhysicalActivePrefix::Preflight(physical,{limits.max_host_bytes},activity);
    if(checked.status!=fe::ActivePrefixStatus::Ok)
      return Error(TransactionStatus::ResourceLimit,checked.message);
  }
  forecast.host_bytes=fixed_host_bytes;
  // The member handle is already counted inside Impl; only its owned backing
  // and retained activity scratch are additional payload.
  if(activity.owned_host_bytes&&!Add(activity.owned_host_bytes-sizeof(fe::PhysicalActivePrefix),forecast.host_bytes))
    return Error(TransactionStatus::ResourceLimit,"Native activity forecast overflows");
  if(!Add(readback_layout.bytes(),forecast.host_bytes)||!Add(inventory.startup_host_bytes,forecast.host_bytes)||!Add(inventory.startup_host_bytes,forecast.host_bytes)||
     !Add(maintenance.startup_host_bytes,forecast.host_bytes)||!Add(maintenance.startup_host_bytes,forecast.host_bytes)||!Add(incidence.host_bytes,forecast.host_bytes)||
     !Add(forecast.host_bytes,forecast.startup_host_bytes)||!Add(upload.bytes+sizeof(Plan),forecast.startup_host_bytes)||
     forecast.startup_host_bytes>limits.max_host_bytes)
    return Error(TransactionStatus::ResourceLimit,"Complete native host forecast exceeds cap");
  return {TransactionStatus::Ok,"OK"};
}
}
TransactionReport PreparePlan(const TransactionConfig& config,const FixedMainSource& source,
    const fe::ShellPhysicalBinding& physical,TransactionLimits limits,std::size_t fixed_host_bytes,Plan& plan) noexcept {
  return Build(config,source,physical,limits,fixed_host_bytes,plan);
}
TransactionReport PreparePlan(const TransactionConfig& config,const MovingMainSource& source,
    const fe::ShellPhysicalBinding& physical,TransactionLimits limits,std::size_t fixed_host_bytes,Plan& plan) noexcept {
  return Build(config,source,physical,limits,fixed_host_bytes,plan);
}
TransactionReport PreparePlan(const TransactionConfig& config,const MixedMovingMainSource& source,
    const fe::ShellPhysicalBinding& physical,TransactionLimits limits,std::size_t fixed_host_bytes,Plan& plan) noexcept {
  return Build(config,source,physical,limits,fixed_host_bytes,plan);
}
} // namespace tlfea::contact::radioss_type25::runtime_detail
namespace tlfea::contact::radioss_type25 {
TransactionReport Transaction::Preflight(const TransactionConfig& config,const FixedMainSource& source,
    const tl::fea::ShellPhysicalBinding& physical,TransactionForecast& output,TransactionLimits limits) noexcept {
  runtime_detail::Plan plan(limits);
  const auto result=runtime_detail::PreparePlan(config,source,physical,limits,sizeof(Transaction)+sizeof(Impl),plan);
  if(result.status==TransactionStatus::Ok)output=plan.forecast;
  return result;
}
TransactionReport Transaction::Preflight(const TransactionConfig& config,const MovingMainSource& source,
    const tl::fea::ShellPhysicalBinding& physical,TransactionForecast& output,TransactionLimits limits) noexcept {
  runtime_detail::Plan plan(limits);
  const auto result=runtime_detail::PreparePlan(config,source,physical,limits,sizeof(Transaction)+sizeof(Impl),plan);
  if(result.status==TransactionStatus::Ok)output=plan.forecast;
  return result;
}
TransactionReport Transaction::Preflight(const TransactionConfig& config,const MixedMovingMainSource& source,
    const tl::fea::ShellPhysicalBinding& physical,TransactionForecast& output,TransactionLimits limits) noexcept {
  runtime_detail::Plan plan(limits);
  const auto result=runtime_detail::PreparePlan(config,source,physical,limits,sizeof(Transaction)+sizeof(Impl),plan);
  if(result.status==TransactionStatus::Ok)output=plan.forecast;
  return result;
}
}
