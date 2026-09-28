// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Storage.h"
#include "ActivityRuntime.h"
namespace tlfea::contact::radioss_type25::runtime_detail {
TransactionReport ActivityReport(tl::fea::PhysicalActivityReport input) noexcept {
  using A=tl::fea::PhysicalActivityStatus;
  TransactionStatus status=TransactionStatus::PublicationFailure;
  switch(input.status) {
    case A::Ok: return {TransactionStatus::Ok,"OK"};
    case A::ResourceLimit: status=TransactionStatus::ResourceLimit;break;
    case A::DeviceFailure: status=TransactionStatus::DeviceFailure;break;
    case A::SourceMismatch: status=TransactionStatus::SourceMismatch;break;
    case A::OwnerFailure: status=TransactionStatus::OwnerFailure;break;
    case A::StaleReceipt: status=TransactionStatus::StaleAttempt;break;
    case A::InvalidActivity: case A::Reactivation: case A::UnsupportedRemoval:
      status=TransactionStatus::ActivityChange;break;
    default: break;
  }
  return {status,input.message,input.family_index};
}
}
namespace tlfea::contact::radioss_type25 {
namespace fe=tl::fea;namespace rd=runtime_detail;
TransactionReport Transaction::Impl::SelectActivity(unsigned slot) noexcept {
  const auto selected=activity->operands.view(slot);
  if(selected.main_count!=source.selection.main_count||selected.primary_count!=source.primary_main_count||
      selected.secondary_count!=source.selection.secondary_count||!selected.mains||
      !selected.main_stiffness_si||!selected.secondary_stiffness_si||
      selected.node_count!=source.selection.node_count||!selected.main_node_activity||
      (device.normal.shape.enabled&&(!selected.normal_mains||!selected.normal_coefficients||
        selected.free_count>device.normal.shape.free_capacity||
        (selected.free_count&&!selected.free_mains))))
    return {TransactionStatus::SourceMismatch,"Native activity operand slot is unavailable"};
  device.source.mains=selected.mains;
  device.main_stiffness=selected.main_stiffness_si;
  device.secondary_stiffness=selected.secondary_stiffness_si;
  if(device.normal.shape.enabled) {
    device.normal.topology.mains=selected.normal_mains;
    device.normal.coefficients=selected.normal_coefficients;
    device.normal.free_mains=selected.free_mains;
    device.normal.shape.free_count=selected.free_count;
  }
  return {TransactionStatus::Ok,"OK"};
}
TransactionReport Transaction::Impl::CaptureAcceptedActivity(const fe::NodalTrialToken& token,
    const fe::NodalAssemblyView& view,const fe::NativeContactPublicationSnapshot& selected) noexcept {
  if(!activity)return {TransactionStatus::Ok,"OK"};
  auto result=rd::ActivityReport(activity->snapshot.CaptureAccepted(*owner,token,view,&activity->accepted));
  if(result.status!=TransactionStatus::Ok)return result;
  fe::PhysicalActivityDeviceView current;
  result=rd::ActivityReport(activity->snapshot.BorrowAccepted(*owner,token,view,activity->accepted,&current));
  if(result.status!=TransactionStatus::Ok)return result;
  // Startup source coefficients describe a wholly active physical model. After
  // epoch zero the mandatory common receipt proves the exact source lifecycle
  // by induction; an unrelated accepted state cannot enter this participant.
  if(!selected.stamp.epoch&&(current.qeph.summary.first_inactive!=SIZE_MAX||
      current.t3.summary.first_inactive!=SIZE_MAX))
    return {TransactionStatus::ActivityChange,"Native shell-removal startup requires active physical parents"};
  return SelectActivity(selected.selectors.activity);
}
TransactionReport Transaction::Impl::StageCandidateActivity(const fe::NodalTrialToken& token,
    const fe::NodalPreparedView& view,const fe::ShellPhysicalDiagnostics& physical) noexcept {
  if(!activity)return {TransactionStatus::Ok,"OK"};
  auto report=rd::ActivityReport(activity->snapshot.CapturePrepared(*owner,token,physical,view,
      activity->accepted,&activity->prepared));
  if(report.status!=TransactionStatus::Ok)return report;
  fe::PhysicalActivityDeviceView current;
  report=rd::ActivityReport(activity->snapshot.BorrowPrepared(*owner,token,physical,view,
      activity->prepared,&current));
  if(report.status!=TransactionStatus::Ok)return report;
  diagnostics.activity_removed_qeph=current.qeph.summary.removed_count;
  diagnostics.activity_removed_t3=current.t3.summary.removed_count;
  diagnostics.activity_first_removed_qeph=current.qeph.summary.first_removed;
  diagnostics.activity_first_removed_t3=current.t3.summary.first_removed;
  if(!current.qeph.summary.removed_count&&!current.t3.summary.removed_count)
    return {TransactionStatus::Ok,"OK"};
  const auto accepted=state.Accepted(*owner);
  if(!accepted.available)return {TransactionStatus::PublicationFailure,"Native activity accepted selector is unavailable"};
  const auto staged=activity->operands.Stage(accepted.selectors.activity,current);
  if(staged.report.status!=TransactionStatus::Ok)return staged.report;
  diagnostics.activity_changed=staged.changed;
  diagnostics.activity_affected_events=staged.affected_events;
  diagnostics.activity_removed_events=staged.removed_events;
  diagnostics.activity_removed_mains=staged.removed_mains;
  diagnostics.activity_orphan_secondaries=staged.orphan_secondaries;
  if(!staged.changed)return {TransactionStatus::Ok,"OK"};
  if(staged.staged_slot!=(accepted.selectors.activity^1u)||
      accepted.selectors.activity_generation==UINT64_MAX)
    return {TransactionStatus::ResourceLimit,"Native activity selector generation cannot advance"};
  const auto next=activity->operands.view(staged.staged_slot);
  if(next.secondary_count!=source.selection.secondary_count||!next.secondary_coefficients)
    return {TransactionStatus::SourceMismatch,"Native staged secondary coefficients are unavailable"};
  auto error=rd::ApplyActivitySecondary(next.secondary_coefficients,
      device.secondary[trial_selectors.history],next.secondary_count,stream);
  const auto drained=cudaStreamSynchronize(stream);
  if(error!=cudaSuccess||drained!=cudaSuccess)
    return {TransactionStatus::DeviceFailure,"Native candidate secondary activity transfer failed"};
  trial_selectors.activity=staged.staged_slot;
  trial_selectors.activity_generation=accepted.selectors.activity_generation+1;
  return {TransactionStatus::Ok,"OK"};
}
}
