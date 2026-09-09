#include "Q4PlanarContactDiagnostics.h"

#include <cmath>
#include <cstring>
#include <new>

namespace tlfea::contact {
namespace {
namespace fea=tl::fea;
namespace detail=q4_planar_detail;
using Code=Q4PlanarContactStatus;
using detail::Storage;

__global__ void ConstructSelectedLeaves(Storage* storage) {
  // Non-allocating placement array construction establishes the selected type
  // before any indexed scratch access. No array of the other backend exists.
  if (storage->model.config.integration_backend == Q4PlanarIntegrationBackend::ScalarDyadicSquares)
    ::new (storage->leaves) Q4IntegrationCell[MaxQ4IntegrationLeaves];
  else if (storage->model.config.integration_backend == Q4PlanarIntegrationBackend::RectangularDyadic)
    ::new (storage->leaves) Q4RectangularCell[MaxQ4IntegrationLeaves];
}
__global__ void MarkFailure(fea::NodalAssemblyView view,std::uint32_t node) {
  fea::RecordNodalAssemblyFailure(view,Status::kInvalidArgument,node);
}
__device__ bool ValidateMass(Storage& storage,const fea::NodalAssemblyView& view) {
  const auto& model=storage.model;
  // At least one mask-6 node is required in each parent by C1/C3. A valid
  // physical owner with component constraints therefore declares this model.
  if (view.mass.model != TranslationMassModel::kUnspecified)
    return detail::Fail(storage.control,Code::InvalidMass);
  for (unsigned i=0;i<model.stiffness.count;++i) {
    const auto node=model.stiffness.nodes[i]; const auto fixed=model.fixed[node];
    if (view.translation_fixed_bits[node] != fixed || view.mass.fixed[node] != (fixed == 7) ||
        view.mass.inverse_mass[node] != model.inverse_mass[node])
      return detail::Fail(storage.control,Code::InvalidMass,UINT32_MAX,node);
  }
  return true;
}
__global__ void AssembleBatch(Storage* storage,fea::NodalAssemblyView view) {
  auto& s=*storage; s.control={}; s.base={};
  if (view.result->base_epoch != view.accepted.base_epoch || view.result->attempt != view.attempt ||
      view.bounds->base_epoch != view.accepted.base_epoch || view.bounds->attempt != view.attempt ||
      !view.bounds->initialized || !view.bounds->valid || view.bounds->sealed || view.result->status != Status::kOk) {
    detail::Fail(s.control,Code::AssemblyFailure);
  } else if (ValidateMass(s,view) && detail::Evaluate(s,view.accepted,view.attempt,Q4PlanarContactPhase::AcceptedBase)) {
    for (unsigned i=0;i<s.model.stiffness.count;++i) {
      const auto node=s.model.stiffness.nodes[i]; const double previous=view.forces.force_x[node];
      const double sum=previous+s.force[i]; Q4IntegralInterval exact_sum;
      if (!IsFinite(view.forces.force_y[node]) || !IsFinite(view.forces.force_z[node]) ||
          !IsFinite(view.forces.couple_x[node]) || !IsFinite(view.forces.couple_y[node]) || !IsFinite(view.forces.couple_z[node]) ||
          !q4_bounds::Add({previous,previous},{s.force[i],s.force[i]},&exact_sum) ||
          !detail::Radius(sum,exact_sum,&s.addition_error[i])) {
        detail::Fail(s.control,Code::AssemblyFailure,UINT32_MAX,node); break;
      }
      s.staged_total[i]=sum;
    }
    if (s.control.status == Code::Ok) {
      for (unsigned i=0;i<s.model.stiffness.count;++i) {
        view.forces.force_x[s.model.stiffness.nodes[i]]=s.staged_total[i];
        s.base.force[i]=s.force[i]; s.base.force_error[i]=s.force_error[i];
        s.base.addition_error[i]=s.addition_error[i];
      }
      s.control.diagnostics.valid=true; s.base.diagnostics=s.control.diagnostics;
    }
  }
  if (s.control.status != Code::Ok) fea::RecordNodalAssemblyFailure(view,Status::kInvalidArgument,s.control.node);
}
__global__ void EvaluatePrepared(Storage* storage,fea::NodalPreparedView view) {
  auto& s=*storage; s.control={};
  const auto& base=s.base.diagnostics;
  if (!base.valid || base.owner_id != view.owner_id || base.base_epoch != view.kinematics.base_epoch ||
      base.attempt != view.attempt || base.configuration_id != s.model.config.configuration_id ||
      base.integration_backend != s.model.config.integration_backend) {
    detail::Fail(s.control,Code::StaleAttempt); return;
  }
  // The owner guarantees immutable mass/masks and a stable accepted base during
  // this prepared view. Candidate evaluation uses the copied, verified masses.
  if (!detail::Evaluate(s,view.kinematics,view.attempt,Q4PlanarContactPhase::PreparedCandidate) ||
      !detail::MeasureInterval(s,view)) return;
  s.control.diagnostics.valid=true;
}
bool Kinematics(const fea::DeviceNodalKinematicsView& view,const Q4PlanarContactConfig& config) {
  return view.position_xyz && view.velocity_xyz && view.node_count == config.owner.node_count &&
         view.base_epoch >= config.owner.epoch;
}
bool Identity(const Q4PlanarContactDiagnostics& a,const Q4PlanarContactDiagnostics& b) {
  return a.valid && b.valid && a.owner_id == b.owner_id && a.base_epoch == b.base_epoch && a.attempt == b.attempt &&
         a.configuration_id == b.configuration_id && a.wall_binding_id == b.wall_binding_id && a.phase == b.phase &&
         a.integration_backend == b.integration_backend;
}
Q4PlanarContactReport Ok() { return {Code::Ok,"OK"}; }
}

Q4PlanarContactReport Q4PlanarContact::Impl::Check(cudaError_t error) {
  if (error == cudaSuccess) return Ok();
  usable=false; has_base=false; has_results=false;
  return {Code::DeviceFailure,cudaGetErrorString(error)};
}
Q4PlanarContactReport Q4PlanarContact::Impl::ConstructLeaves() {
  auto report=Check(cudaGetLastError()); if (report.status != Code::Ok) return report;
  ConstructSelectedLeaves<<<1,1>>>(device);
  report=Check(cudaGetLastError()); if (report.status != Code::Ok) return report;
  return Check(cudaStreamSynchronize(nullptr));
}
Q4PlanarContactReport Q4PlanarContact::Impl::ReadControl(cudaStream_t stream) {
  auto report=Check(cudaGetLastError()); if (report.status != Code::Ok) return report;
  report=Check(cudaMemcpyAsync(&control,&device->control,sizeof(control),cudaMemcpyDeviceToHost,stream));
  if (report.status != Code::Ok) return report;
  report=Check(cudaStreamSynchronize(stream)); if (report.status != Code::Ok) return report;
  return {control.status,control.status == Code::Ok ? "OK" : "Q4 contact evaluation rejected",
          control.parent,control.node,control.geometry,control.integration};
}
Q4PlanarContactReport Q4PlanarContact::Impl::FailAssembly(const fea::NodalAssemblyView& view,Q4PlanarContactReport report) {
  has_base=false; has_results=false;
  if (view.result && view.bounds) {
    MarkFailure<<<1,1,0,view.stream>>>(view,report.node);
    auto failure=Check(cudaGetLastError()); if (failure.status != Code::Ok) return failure;
    failure=Check(cudaStreamSynchronize(view.stream)); if (failure.status != Code::Ok) return failure;
  }
  return report;
}
Q4PlanarContactReport Q4PlanarContact::Assemble(const fea::NodalAssemblyView& view,Q4PlanarContactDiagnostics* output) {
  if (!impl_) {
    Impl failure;
    return failure.FailAssembly(view,{Code::NotInitialized,"Q4 contact is not initialized"});
  }
  auto& s=*impl_; s.has_base=false; s.has_results=false;
  if (!s.usable) return s.FailAssembly(view,{Code::DeviceFailure,"Q4 contact CUDA storage is poisoned"});
  if (!output || !Kinematics(view.accepted,s.config) || !view.mass.inverse_mass || !view.mass.fixed ||
      !view.translation_fixed_bits || !view.bounds || !view.result ||
      !view.forces.force_x || !view.forces.force_y || !view.forces.force_z ||
      !view.forces.couple_x || !view.forces.couple_y || !view.forces.couple_z ||
      view.mass.node_count != s.config.owner.node_count || view.forces.node_count != s.config.owner.node_count)
    return s.FailAssembly(view,{Code::InvalidInput,"Invalid Q4 contact assembly/output view"});
  if (view.owner_id != s.config.owner.owner_id)
    return s.FailAssembly(view,{Code::WrongOwner,"Q4 contact assembly belongs to another owner"});
  if (!view.attempt || view.accepted.base_epoch != view.mass.base_epoch || view.accepted.base_epoch != view.forces.base_epoch ||
      view.accepted.base_epoch < s.last_epoch || view.attempt <= s.last_attempt)
    return s.FailAssembly(view,{Code::StaleAttempt,"Duplicate or stale Q4 contact contribution"});
  // Drain a pending launch error before this contribution can scatter forces.
  // Recoverable CUDA failures still make the owner's assembly fail sticky.
  auto report=s.Check(cudaGetLastError());
  if (report.status != Code::Ok) return s.FailAssembly(view,report);
  s.last_epoch=view.accepted.base_epoch; s.last_attempt=view.attempt; s.last_stream=view.stream;
  AssembleBatch<<<1,1,0,view.stream>>>(s.device,view);
  report=s.ReadControl(view.stream); if (report.status != Code::Ok) return s.FailAssembly(view,report);
  s.has_base=true; s.has_results=true; *output=s.control.diagnostics;
  return Ok();
}
Q4PlanarContactReport Q4PlanarContact::EvaluateCandidate(const fea::NodalPreparedView& view,Q4PlanarContactDiagnostics* output) {
  if (!impl_) return {Code::NotInitialized,"Q4 contact is not initialized"};
  auto& s=*impl_; s.has_results=false;
  if (!s.usable) return {Code::DeviceFailure,"Q4 contact CUDA storage is poisoned"};
  if (!output || !Kinematics(view.kinematics,s.config) || !Kinematics(view.base_kinematics,s.config) ||
      !std::isfinite(view.proposed_time) || view.proposed_time <= s.config.owner.time)
    return {Code::InvalidInput,"Invalid Q4 contact prepared/base/output view"};
  if (view.owner_id != s.config.owner.owner_id) return {Code::WrongOwner,"Q4 contact candidate belongs to another owner"};
  if (!s.has_base || view.attempt != s.last_attempt || view.kinematics.base_epoch != s.last_epoch ||
      view.base_kinematics.base_epoch != s.last_epoch || view.stream != s.last_stream)
    return {Code::StaleAttempt,"Q4 contact candidate does not match its base assembly"};
  EvaluatePrepared<<<1,1,0,view.stream>>>(s.device,view);
  auto report=s.ReadControl(view.stream); if (report.status != Code::Ok) return report;
  s.has_results=true; *output=s.control.diagnostics;
  return Ok();
}
Q4PlanarContactReport Q4PlanarContact::CopyParentResults(const Q4PlanarContactDiagnostics& expected,
                                                       Q4PlanarParentResult* output,std::size_t capacity) {
  if (!impl_) return {Code::NotInitialized,"Q4 contact is not initialized"};
  auto& s=*impl_;
  if (!s.usable) return {Code::DeviceFailure,"Q4 contact CUDA storage is poisoned"};
  if (!s.has_results || !Identity(expected,s.control.diagnostics)) return {Code::StaleAttempt,"Stale Q4 contact result identity/phase"};
  if (!output) return {Code::InvalidInput,"Missing Q4 contact result output"};
  if (capacity < s.parent_count) return {Code::ResourceLimit,"Insufficient Q4 contact result capacity"};
  const auto bytes=s.parent_count*sizeof(Q4PlanarParentResult);
  const auto begin=reinterpret_cast<std::uintptr_t>(output),identity=reinterpret_cast<std::uintptr_t>(&expected);
  if (bytes > UINTPTR_MAX-begin || sizeof(expected) > UINTPTR_MAX-identity ||
      (begin < identity+sizeof(expected) && identity < begin+bytes))
    return {Code::InvalidInput,"Q4 contact result range overflows or aliases its identity"};
  auto report=s.Check(cudaGetLastError()); if (report.status != Code::Ok) return report;
  report=s.Check(cudaMemcpyAsync(s.staging.data(),s.device->result,bytes,cudaMemcpyDeviceToHost,s.last_stream));
  if (report.status != Code::Ok) return report;
  report=s.Check(cudaStreamSynchronize(s.last_stream)); if (report.status != Code::Ok) return report;
  std::memcpy(output,s.staging.data(),bytes); return Ok();
}
}  // namespace tlfea::contact
