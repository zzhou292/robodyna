// SPDX-License-Identifier: AGPL-3.0-or-later
#include "ActivityPlanning.h"
#include "ActivityRuntime.h"
#include <algorithm>
#include <type_traits>
namespace tlfea::contact::radioss_type25::runtime_detail {
namespace {
bool Add(std::size_t value,std::size_t& total) noexcept {
  if(value>SIZE_MAX-total)return false;
  total+=value;return true;
}
template<class Source>
TransactionReport Prepare(const TransactionConfig& config,const Source& source,
    const tl::fea::ShellPhysicalBinding& physical,TransactionLimits limits,
    const SourceStaging& upload,ActivityPlan& result) noexcept {
  if(config.activity!=ContactActivityPolicy::ShellRemoval)return {TransactionStatus::Ok,"OK"};
  if(!source.activity_controls)return {TransactionStatus::InvalidInput,"Shell removal needs declared source controls"};
  result.source_limits.output_bytes=std::min(result.source_limits.output_bytes,limits.max_host_bytes);
  result.source_limits.startup_bytes=std::min(result.source_limits.startup_bytes,limits.max_host_bytes);
  result.snapshot_limits.max_host_bytes=std::min(result.snapshot_limits.max_host_bytes,limits.max_host_bytes);
  result.snapshot_limits.max_startup_host_bytes=std::min(result.snapshot_limits.max_startup_host_bytes,limits.max_host_bytes);
  result.snapshot_limits.max_device_bytes=std::min(result.snapshot_limits.max_device_bytes,limits.max_device_bytes);
  result.operand_limits.max_host_bytes=std::min(result.operand_limits.max_host_bytes,limits.max_host_bytes);
  result.operand_limits.max_startup_host_bytes=std::min(result.operand_limits.max_startup_host_bytes,limits.max_host_bytes);
  result.operand_limits.max_device_bytes=std::min(result.operand_limits.max_device_bytes,limits.max_device_bytes);
  activity_source::PhysicalSources actual{physical,source.activity_type45};
  TransactionReport report;
  if constexpr(std::is_same_v<Source,MixedMovingMainSource>)
    report=result.source.Initialize(actual,source.starter,*source.activity_controls,result.source_limits);
  else report=result.source.Initialize(actual,source,*source.activity_controls,result.source_limits);
  if(report.status!=TransactionStatus::Ok)return report;
  report=ActivityReport(tl::fea::PhysicalActivitySnapshot::Preflight(physical,result.snapshot_limits,result.snapshot));
  if(report.status!=TransactionStatus::Ok)return report;
  const auto& moving=upload.moving;
  result.operands=activity_operands::State::Preflight(result.source,source,
      moving.enabled?&moving.topology:nullptr,config.units,
      ActivitySlotShape(source,moving.enabled,moving.free_main_ids.size()),result.operand_limits);
  if(result.operands.report.status!=TransactionStatus::Ok)return result.operands.report;
  // Source construction completes first. Snapshot initialization then drains
  // its law-byte staging before operand upload begins. The operand startup
  // forecast already includes the retained source Plan; do not count it twice.
  result.host_bytes=sizeof(ActivityRuntime);
  result.device_bytes=result.snapshot.device_bytes;
  auto snapshot_peak=result.source.forecast().output_bytes;
  auto operand_peak=result.operands.startup_host_bytes;
  if(!Add(result.snapshot.owned_host_bytes,result.host_bytes)||
      !Add(result.snapshot.preparation_host_bytes,result.host_bytes)||
      !Add(result.operands.owned_host_bytes,result.host_bytes)||
      !Add(result.snapshot.startup_host_bytes,snapshot_peak)||
      !Add(result.snapshot.owned_host_bytes,operand_peak)||
      !Add(result.operands.owned_device_bytes,result.device_bytes))
    return {TransactionStatus::ResourceLimit,"Native activity allocation forecast overflows"};
  result.startup_host_bytes=std::max({result.source.forecast().startup_bytes,snapshot_peak,operand_peak});
  if(!Add(sizeof(ActivityRuntime),result.startup_host_bytes))
    return {TransactionStatus::ResourceLimit,"Native activity startup forecast overflows"};
  result.startup_host_bytes=std::max(result.startup_host_bytes,result.host_bytes);
  if(result.host_bytes>limits.max_host_bytes||result.startup_host_bytes>limits.max_host_bytes||
      result.device_bytes>limits.max_device_bytes)
    return {TransactionStatus::ResourceLimit,"Complete native activity allocation forecast exceeds cap"};
  return {TransactionStatus::Ok,"OK"};
}
}
activity_operands::BorrowedSlot ActivitySlotShape(const ContactSourceInput& source,bool normals,
    std::size_t free_count) noexcept {
  activity_operands::BorrowedSlot slot;
  slot.main_capacity=source.selection.main_count;
  slot.normal_capacity=normals?slot.main_capacity:0;
  slot.free_capacity=normals?slot.main_capacity:0;
  slot.primary_capacity=source.primary_main_count;
  slot.secondary_capacity=source.selection.secondary_count;
  slot.initial_free_count=free_count;return slot;
}
activity_operands::BorrowedSlot ActivitySlot(void* arena,const Layout& layout,
    const ContactSourceInput& source,NormalShape normal) noexcept {
  auto slot=ActivitySlotShape(source,normal.enabled,normal.free_count);
  slot.mains=tl::util::ArenaPointer<lifecycle::Main>(arena,layout.mains);
  slot.main_stiffness_si=tl::util::ArenaPointer<double>(arena,layout.main_stiffness);
  slot.secondary_stiffness_si=tl::util::ArenaPointer<double>(arena,layout.secondary_stiffness);
  if(normal.enabled) {
    slot.normal_mains=tl::util::ArenaPointer<startup::Main>(arena,layout.normal.topology);
    slot.normal_coefficients=tl::util::ArenaPointer<double>(arena,layout.normal.coefficients);
    slot.free_mains=tl::util::ArenaPointer<std::uint32_t>(arena,layout.normal.free_mains);
    slot.free_capacity=layout.normal.free_mains.bytes/sizeof(std::uint32_t);
  }
  return slot;
}
#define PREPARE(Source) \
TransactionReport PrepareActivity(const TransactionConfig& c,const Source& s, \
    const tl::fea::ShellPhysicalBinding& p,TransactionLimits l,const SourceStaging& u,ActivityPlan& a) noexcept { \
  return Prepare(c,s,p,l,u,a); \
}
PREPARE(FixedMainSource)
PREPARE(MovingMainSource)
PREPARE(MixedMovingMainSource)
#undef PREPARE
}
