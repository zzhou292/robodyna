#include "ExplicitNodalStep.h"
#include "FENodalStateStorage.h"
#include "NodalNodeStep.h"

namespace tl::fea {
namespace {
using nodal_detail::Phase;
namespace sc = tlfea::contact;

// Same bounded single-writer execution as the existing translational gate.
// This operation consumes one sealed force assembly and owns no physical state.
__global__ void Advance(nodal_detail::Control* control, const double* accepted,
                        double* trial, const double* force, const double* inverse,
                        const std::uint8_t* constraints, std::uint32_t n, double h, double kick_dt,
                        double maximum_angle, std::uint64_t epoch, std::uint64_t attempt,
                        const std::uint8_t* rotation_present) {
  if (control->status != NodalStatus::Ok) return;
  if (control->rows.base_epoch != epoch || control->rows.attempt != attempt ||
      !stability::IsCurrentLimit(control->rows, control->limit) || control->limit.dt < h) {
    control->status = NodalStatus::StaleTrial; return;
  }
  if (control->limit.has_stiffness_or_damping) {
    control->status = NodalStatus::MissingStepAdmission; return;
  }
  for (std::uint32_t i = 0; i < n; ++i) {
    const auto status=nodal_detail::AdvanceOrdinaryNode(accepted,trial,force,inverse,constraints,i,n,h,kick_dt,maximum_angle,
        nullptr,nullptr,rotation_present);
    if(status!=NodalStatus::Ok) {
      if(status==NodalStatus::StepTooLarge) control->limit.dt=0;
      control->status=status; control->node=i; return;
    }
  }
}
}  // namespace

NodalReport AdvanceNodal(FENodalState& owner, const NodalTrialToken& token, const NodalStepAdmission& admission) {
  if (!owner.impl_) return {NodalStatus::NotInitialized, "Owner is not initialized"};
  return owner.impl_->AdvanceSealedNodal(token.owner_id_,token.base_epoch_,token.attempt_,admission,
                                       NodalTemporalScheme::VelocityFirst);
}

NodalReport AdvanceStaggeredPrescribed(FENodalState& owner, const NodalTrialToken& token,
                                      const NodalStaggeredPrescribedAdmission& admission) {
  if (!owner.impl_) return {NodalStatus::NotInitialized, "Owner is not initialized"};
  const NodalStepAdmission constant{admission.owner_id,admission.base_epoch,admission.attempt,
      admission.maximum_dt,admission.maximum_rotation_increment,NodalStepAdmissionKind::PrescribedConstantLoads};
  return owner.impl_->AdvanceSealedNodal(token.owner_id_,token.base_epoch_,token.attempt_,constant,
                                       NodalTemporalScheme::StaggeredHalfKickStart);
}

NodalReport AdvanceStaggeredHistory(FENodalState& owner, const NodalTrialToken& token,
                                   const NodalStaggeredHistoryAdmission& admission) {
  if (!owner.impl_) return {NodalStatus::NotInitialized, "Owner is not initialized"};
  const NodalStepAdmission history{admission.owner_id,admission.base_epoch,admission.attempt,
      admission.maximum_dt,admission.maximum_rotation_increment,
      NodalStepAdmissionKind::RestrictedHistoryTrajectory,admission.qualification_id,0};
  return owner.impl_->AdvanceSealedNodal(token.owner_id_,token.base_epoch_,token.attempt_,history,
                                       NodalTemporalScheme::StaggeredHalfKickStart);
}

NodalReport FENodalState::Impl::AdvanceSealedNodal(
    std::uint64_t owner_id, std::uint64_t epoch, std::uint64_t attempt,
    const NodalStepAdmission& admission, NodalTemporalScheme expected_scheme, bool with_rigid_groups, bool with_cin) {
  auto& s = *this;
  if (!s.usable) return {NodalStatus::DeviceFailure, "CUDA owner is poisoned"};
  if (!s.Matches(owner_id, epoch, attempt))
    return s.Reject(NodalStatus::StaleTrial, "Trial token belongs to another owner or attempt");
  if (s.phase != Phase::Sealed) return s.Reject(NodalStatus::WrongPhase, "Assembly has not been sealed");
  if (bool(s.cin) != with_cin)
    return s.Reject(NodalStatus::MissingStepAdmission, "CIN attachments require their dedicated advance operation");
  if(bool(s.rigid_groups)!=with_rigid_groups)
    return s.Reject(NodalStatus::MissingStepAdmission,"Attached rigid groups require their dedicated advance operation");
  if (s.config.temporal_scheme != expected_scheme)
    return s.Reject(NodalStatus::UnsupportedTemporalScheme, "Step operation does not match the owner's temporal scheme");
  if (!s.has_rotations)
    return s.Reject(NodalStatus::UnsupportedRotation, "AdvanceNodal requires extended nodal initialization");
  const bool elastic = admission.kind == NodalStepAdmissionKind::RestrictedElasticTrajectory;
  const bool history = admission.kind == NodalStepAdmissionKind::RestrictedHistoryTrajectory;
  if(with_rigid_groups&&!history)
    return s.Reject(NodalStatus::MissingStepAdmission,"Rigid recurrence requires case-qualified history admission");
  if ((history && expected_scheme != NodalTemporalScheme::StaggeredHalfKickStart) ||
      (elastic && expected_scheme != NodalTemporalScheme::VelocityFirst))
    return s.Reject(NodalStatus::UnsupportedTemporalScheme, "Restricted admission does not match this step operation");
  if (admission.kind != NodalStepAdmissionKind::PrescribedConstantLoads && !elastic && !history)
    return s.Reject(NodalStatus::MissingStepAdmission, "Missing declared nodal step policy");
  if (admission.owner_id != s.stamp.owner_id || admission.base_epoch != s.stamp.epoch || admission.attempt != s.attempt)
    return s.Reject(NodalStatus::StaleTrial, "Step admission belongs to another owner or attempt");
  constexpr double pi = 3.14159265358979323846;
  if (!std::isfinite(admission.maximum_dt) || admission.maximum_dt <= 0 ||
      !std::isfinite(admission.maximum_rotation_increment) || admission.maximum_rotation_increment <= 0 ||
      admission.maximum_rotation_increment >= pi)
    return s.Reject(NodalStatus::InvalidInput, "Invalid explicit step or rotation increment bound");
  if (admission.maximum_dt < s.config.fixed_dt) {
    auto report = s.Reject(NodalStatus::StepTooLarge, "Fixed step exceeds explicit admission");
    report.stable_dt = admission.maximum_dt;
    return report;
  }
  if (elastic) {
    if (!admission.qualification_id || !std::isfinite(admission.stiffness_rate_envelope) ||
        admission.stiffness_rate_envelope <= 0)
      return s.Reject(NodalStatus::MissingStepAdmission, "Restricted elasticity requires a qualified all-DOF envelope");
    // Conservative experimental sampling margin, not the translation-row proof.
    const double sampled_limit = .1 / std::sqrt(admission.stiffness_rate_envelope);
    if (s.config.fixed_dt > sampled_limit) {
      auto r = s.Reject(NodalStatus::StepTooLarge, "Fixed step exceeds the sampled elastic envelope");
      r.stable_dt = sampled_limit; return r;
    }
  } else if (history) {
    if (!admission.qualification_id || admission.stiffness_rate_envelope != 0)
      return s.Reject(NodalStatus::MissingStepAdmission, "History recurrence requires its own qualification identity");
  } else if (admission.qualification_id || admission.stiffness_rate_envelope != 0) {
    return s.Reject(NodalStatus::InvalidInput, "Elastic qualification fields supplied for constant loads");
  }
  cudaError_t error;
  if(with_cin) error=s.LaunchCinAdvance(admission.maximum_rotation_increment);
  else if(with_rigid_groups) error=s.LaunchRigidAdvance(admission.maximum_rotation_increment);
  else {
    Advance<<<1,1,0,s.stream>>>(s.control, s.accepted, s.trial, s.scratch, s.inverse, s.fixed,
        static_cast<std::uint32_t>(s.config.node_count), s.config.fixed_dt, s.candidate_kick_dt, admission.maximum_rotation_increment,
        s.stamp.epoch, s.attempt,s.stamp.has_rotation_presence?s.fixed+3*s.config.node_count:nullptr);
    error=cudaGetLastError();
  }
  auto report = s.Check(error); if (report.status != NodalStatus::Ok) return report;
  report = s.SynchronizeControl(); if (report.status != NodalStatus::Ok) return report;
  s.pending_qualification = elastic || history ? admission.qualification_id : 0;
  s.phase = elastic || history ? Phase::AwaitingValidation : Phase::Ready;
  return {NodalStatus::Ok, "OK"};
}

NodalReport CompleteNodalValidation(FENodalState& owner, const NodalTrialToken& token,
                                    const NodalValidationReceipt& receipt) {
  if (!owner.impl_) return {NodalStatus::NotInitialized, "Owner is not initialized"};
  auto& s = *owner.impl_;
  if (!s.usable) return {NodalStatus::DeviceFailure, "CUDA owner is poisoned"};
  if (!s.Matches(token.owner_id_, token.base_epoch_, token.attempt_) ||
      receipt.owner_id != s.stamp.owner_id || receipt.base_epoch != s.stamp.epoch || receipt.attempt != s.attempt)
    return s.Reject(NodalStatus::StaleTrial, "Candidate receipt belongs to another owner or attempt");
  if (s.phase != Phase::AwaitingValidation)
    return s.Reject(NodalStatus::WrongPhase, "No candidate awaiting restricted validation");
  if (!receipt.passed || !receipt.qualification_id || receipt.qualification_id != s.pending_qualification)
    return s.Reject(NodalStatus::MissingCandidateValidation, "Missing or mismatched candidate qualification");
  auto report = s.Check(cudaGetLastError()); if (report.status != NodalStatus::Ok) return report;
  report = s.Check(cudaStreamSynchronize(s.stream)); if (report.status != NodalStatus::Ok) return report;
  s.phase = Phase::Ready;
  return {NodalStatus::Ok, "Candidate validation completed"};
}

}  // namespace tl::fea
