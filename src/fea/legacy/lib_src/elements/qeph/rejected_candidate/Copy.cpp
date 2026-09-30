// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Read.h"
#include "../QephBatchStorage.h"
#include "../QephHistory.h"
#include "../../ShellBatchFields.h"
#include "lib_src/solvers/NodalNativePhysicalCoefficients.h"
namespace tl::fea::qeph {
namespace {
bool SameReport(const BatchReport& r,const batch_detail::Control& c) noexcept {
  return r.status==c.status&&r.element==c.element&&r.node==c.node&&
      r.element_status==c.element_status&&r.nodal_status==NodalStatus::Ok;
}
RejectedCandidateMetadata Metadata(const NodalStamp& s,const BatchDiagnostics& accepted,
    const batch_detail::Control& c) noexcept {
  RejectedCandidateMetadata out;
  out.original={c.status,c.element,c.node,c.element_status,NodalStatus::Ok};
  out.owner={s.owner_id,s.epoch,s.node_count,s.time,s.fixed_dt,s.velocity_time,s.temporal_scheme,s.velocity_phase};
  out.accepted=accepted;out.candidate=c.diagnostics;return out;
}
}
RejectedCaptureReport QephBatch::CopyRejectedCandidate(FENodalState& owner,const NodalTrialToken& token,
    const NodalPreparedView& view,const BatchReport& expected,RejectedCandidateInput* output) noexcept {
  RejectedCaptureReport report;
  auto fail=[&](RejectedCaptureStatus status,const char* message) {report.status=status;report.message=message;return report;};
  if(!impl_||!output)return fail(RejectedCaptureStatus::InvalidInput,"Missing diagnostic batch/output");
  auto& s=*impl_;using trial_identity::Disjoint;
  if(!s.OutputDisjoint(output,sizeof(*output))||!Disjoint(output,sizeof(*output),this,sizeof(*this))||
     !Disjoint(output,sizeof(*output),&s,sizeof(s))||!Disjoint(output,sizeof(*output),&owner,sizeof(owner))||
     !Disjoint(output,sizeof(*output),&token,sizeof(token))||!Disjoint(output,sizeof(*output),&view,sizeof(view))||
     !Disjoint(output,sizeof(*output),&expected,sizeof(expected)))
    return fail(RejectedCaptureStatus::InvalidInput,"Diagnostic output aliases owner/source/authentication input");
  if(!s.usable||!s.bound||s.pending||!s.rejected_candidate)
    return fail(RejectedCaptureStatus::NoRejectedCandidate,"No latched failed QEPH candidate");
  if(!SameReport(expected,s.control)||!batch_detail::SameDiagnostics(s.candidate_diagnostics,s.control.diagnostics)||
     !trial_identity::SamePrepared(view,s.candidate_view)||view.attempt!=s.last_candidate_attempt)
    return fail(RejectedCaptureStatus::StaleTrial,"Failed report/view differs from the actual rejected attempt");
  const auto authenticated=native_physical_coefficients::AuthenticatePrepared(owner,token,s.accepted_stamp,view);
  if(authenticated.status!=NodalStatus::Ok)
    return fail(RejectedCaptureStatus::StaleTrial,"Rejected candidate is no longer the live owner's prepared attempt");
  if(s.control.diagnostics.valid||s.control.diagnostics.phase!=BatchPhase::Prepared||
      s.control.diagnostics.attempt!=view.attempt||s.control.diagnostics.base_epoch!=s.accepted_stamp.epoch)
    return fail(RejectedCaptureStatus::StaleTrial,"Rejected candidate diagnostics have no failed prepared identity");
  rejected_detail::Reader read{view.stream};batch_detail::Control actual;
  if(!read.Value(actual,&s.storage->control)) {
    report.cuda_status=read.error;return fail(RejectedCaptureStatus::DeviceFailure,"Rejected control diagnostic read failed");
  }
  if(!SameReport(expected,actual)||!batch_detail::SameDiagnostics(actual.diagnostics,s.control.diagnostics))
    return fail(RejectedCaptureStatus::StaleTrial,"Device control changed after the failed candidate");
  report.metadata=Metadata(s.accepted_stamp,s.accepted_diagnostics,actual);report.metadata_available=true;
  if(actual.element>=s.config.element_count||
     (actual.status!=BatchStatus::ElementFailure&&actual.status!=BatchStatus::NonfiniteResult))
    return fail(RejectedCaptureStatus::UnsupportedFailure,"Failure has no authenticated parent input; aggregate capture unavailable");
  const auto parent=static_cast<std::size_t>(actual.element);Status worker;
  if(!read.Value(worker,s.device_header.candidate_status+parent)) {
    report.cuda_status=read.error;return fail(RejectedCaptureStatus::DeviceFailure,"Rejected element status read failed");
  }
  if((actual.status==BatchStatus::ElementFailure&&worker!=actual.element_status)||
     (actual.status==BatchStatus::NonfiniteResult&&worker!=Status::kSuccess))
    return fail(RejectedCaptureStatus::StaleTrial,"Parent worker status no longer matches the rejection");
  RejectedCandidateInput staged;staged.metadata=report.metadata;staged.mapped=s.physical.has_value();
  if(!read.Value(staged.element,s.device_header.model.element+parent)||
     !read.Value(staged.accepted_force,s.device_header.slab[s.AcceptedSlabIndex()].element+parent)) {
    report.cuda_status=read.error;return fail(RejectedCaptureStatus::DeviceFailure,"Accepted parent diagnostic read failed");
  }
  const auto& history=staged.accepted_force.proposed_history;
  if(!history.prepared()||!history.matches_reference(staged.element.reference)||
     !detail::ValidHistoryValues(history.data(),true)||history.stamp().time!=view.base_time||
     history.stamp().sample_index!=s.accepted_stamp.epoch)
    return fail(RejectedCaptureStatus::InvalidSource,"Accepted history cannot be persisted without changing its binding");
  double x[12],v[12],omega[12];
  for(unsigned k=0;k<4;++k) {
    const auto node=staged.element.nodes[k];
    if(node>=s.accepted_stamp.node_count)return fail(RejectedCaptureStatus::InvalidSource,"Parent has an out-of-domain node");
    if(!read.Bytes(x+3*k,view.kinematics.position_xyz+3*node,3*sizeof(double))||
       !read.Bytes(v+3*k,view.kinematics.velocity_xyz+3*node,3*sizeof(double))||
       !read.Bytes(omega+3*k,view.kinematics.angular_velocity_xyz+3*node,3*sizeof(double))) {
      report.cuda_status=read.error;return fail(RejectedCaptureStatus::DeviceFailure,"Prepared parent node diagnostic read failed");
    }
  }
  DeviceNodalKinematicsView fields{};fields.position_xyz=x;fields.velocity_xyz=v;fields.angular_velocity_xyz=omega;
  const std::size_t local[4]{0,1,2,3};
  shell_batch_fields::Gather(local,fields,staged.interval.position_endpoint,staged.interval.velocity_midpoint,staged.interval.omega_midpoint);
  staged.interval.base_time=view.base_time;staged.interval.dt=s.accepted_stamp.fixed_dt;
  staged.interval.sample_index=s.accepted_stamp.epoch+1;
  if(!rejected_detail::ReadMaterial(s.plasticity.get(),parent,s.AcceptedSlabIndex(),staged,read)) {
    report.cuda_status=read.error;
    return fail(read.error==cudaSuccess?RejectedCaptureStatus::InvalidSource:RejectedCaptureStatus::DeviceFailure,
                "Resident material/section diagnostic source is unavailable or outside its owned bounds");
  }
  if(s.joined_binding) {
    staged.source.parent=s.joined_binding->qeph_source_id(parent);
    if(const auto* catalog=s.plasticity?s.plasticity->section_catalog():nullptr) {
      for(std::size_t k=0;k<catalog->parent_count();++k) {
        const auto* row=catalog->parent(k);
        if(row&&row->family==ShellBindingFamily::Qeph&&row->family_index==parent) {
          if(row->source_parent_id!=staged.source.parent)
            return fail(RejectedCaptureStatus::InvalidSource,"Diagnostic source parent identity differs");
          staged.source={true,row->source_parent_id,row->source_part_id,row->material_id,row->section_id};break;
        }
      }
    }
  }
  // No owner/history/publication state is modified. The app's opt-in sink owns
  // once-only capture and persistence; another caller can copy this same view.
  *output=staged;report.status=RejectedCaptureStatus::Captured;report.message="OK";return report;
}
} // namespace tl::fea::qeph
