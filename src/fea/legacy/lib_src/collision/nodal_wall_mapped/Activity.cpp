// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Storage.h"
#include "lib_src/solvers/NodalTrialIdentity.h"
namespace tlfea::contact {
namespace fe=tl::fea;
using Code=NodalWallDeviceStatus;
NodalWallDeviceReport NodalWallMappedContact::Impl::Authenticate(fe::FENodalState& actual) const noexcept {
  if(&actual!=owner) return {Code::WrongOwner,"Mapped wall is bound to another actual owner"};
  const auto checked=publication->ValidatePhysicalSources(actual,physical,participants,identity);
  if(checked.status!=fe::ShellPublicationStatus::Success)
    return {checked.status==fe::ShellPublicationStatus::DeviceFailure?Code::DeviceFailure:Code::StaleAttempt,checked.message};
  return {Code::Ok,"Actual physical activity source authenticated"};
}
NodalWallDeviceReport NodalWallMappedContact::Impl::CaptureActivity(const fe::NodalTrialToken* token,
    const fe::ShellPhysicalDiagnostics* candidate) {
  auto report=Authenticate(*owner);
  if(report.status!=Code::Ok) return report;
  const auto stamp=owner->accepted();
  const auto& binding=*physical.shells();
  const auto q=binding.qeph_count(),t=binding.t3_count(),b=binding.qbat_count();
  if(candidate && (!token || !candidate->valid ||
      !fe::trial_identity::SameStamp(candidate->base_stamp,stamp) ||
      candidate->has_qeph!=bool(q) || candidate->has_t3!=bool(t) || candidate->has_qbat!=bool(b)))
    return {Code::StaleAttempt,"Prepared activity does not cover the common shell scope"};
  if(q) {
    fe::qeph::BatchDiagnostics actual;
    const auto read=candidate?participants.qeph->CopyPreparedParentActivity(*owner,*token,
        candidate->qeph,activity.data(),q):participants.qeph->CopyAcceptedParentActivity(stamp,activity.data(),q,&actual);
    if(read.status!=fe::qeph::BatchStatus::Success)
      return {read.status==fe::qeph::BatchStatus::DeviceFailure?Code::DeviceFailure:Code::StaleAttempt,read.message};
  }
  if(t) {
    fe::t3::BatchDiagnostics actual;
    const auto read=candidate?participants.t3->CopyPreparedParentActivity(*owner,*token,
        candidate->t3,activity.data()+q,t):participants.t3->CopyAcceptedParentActivity(stamp,activity.data()+q,t,&actual);
    if(read.status!=fe::t3::BatchStatus::Success)
      return {read.status==fe::t3::BatchStatus::DeviceFailure?Code::DeviceFailure:Code::StaleAttempt,read.message};
  }
  if(b) {
    fe::qbat::BatchDiagnostics actual;
    const auto read=candidate?participants.qbat->CopyPreparedParentActivity(*owner,*token,
        candidate->qbat,activity.data()+q+t,b):participants.qbat->CopyAcceptedParentActivity(stamp,activity.data()+q+t,b,&actual);
    if(read.status!=fe::qbat::BatchStatus::Success)
      return {read.status==fe::qbat::BatchStatus::DeviceFailure?Code::DeviceFailure:Code::StaleAttempt,read.message};
  }
  auto* output=candidate?local.proposed:local.accepted;
  std::size_t active=0;
  for(std::size_t p=0;p<parents.size();++p) {
    const auto& parent=parents[p];
    const auto offset=parent.family==fe::ShellBindingFamily::T3?q:
        (parent.family==fe::ShellBindingFamily::Qbat?q+t:0);
    const auto flag=activity[offset+parent.index];
    if(flag>1 || (candidate && flag>local.accepted[p]))
      return {Code::InvalidInput,"Invalid activity byte or unsupported inactive-to-active transition",UINT32_MAX,static_cast<std::uint32_t>(p)};
    output[p]=flag;
    active+=flag;
  }
  if(candidate) proposed_active=active;
  else accepted_active=active;
  if(device_sidecar) return Check(cudaMemcpyAsync(candidate?remote.proposed:remote.accepted,
      output,parents.size(),cudaMemcpyHostToDevice,stream));
  return {Code::Ok,"Actual typed activity staged"};
}
NodalWallDeviceReport NodalWallMappedContact::Impl::CaptureBodies() {
  if(!snapshots.size()) return {Code::Ok,"No rigid body response required"};
  fe::NodalStamp stamp;
  const auto copied=owner->CopyAcceptedRigidGroups({snapshots.data(),snapshots.size()},&stamp);
  if(copied.status!=fe::NodalStatus::Ok)
    return {copied.status==fe::NodalStatus::DeviceFailure?Code::DeviceFailure:Code::WrongOwner,copied.message};
  if(!fe::trial_identity::SameStamp(stamp,owner->accepted()))
    return {Code::StaleAttempt,"Rigid contact snapshot endpoint differs"};
  const double drift=stamp.epoch?stamp.fixed_dt:0;
  for(std::size_t g=0;g<snapshots.size();++g) {
    const auto& actual=snapshots[g];
    const auto& source=rigid.groups()[g];
    if(actual.source_kind!=source.source_kind || actual.source_group_id!=source.source_id ||
        actual.source_node_set_id!=source.source_node_set_id ||
        PrepareRigidContactBodyFromAccepted(actual.state,source.mass_kg,source.principal.inertia,
            drift,local.bodies[g])!=Status::kOk)
      return {Code::InvalidMass,"Actual current force-frame rigid response is unavailable"};
  }
  if(device_sidecar) return Check(cudaMemcpyAsync(remote.bodies,local.bodies,layout.bodies.bytes,
      cudaMemcpyHostToDevice,stream));
  return {Code::Ok,"Authentic rigid current force frames staged"};
}
} // namespace tlfea::contact
