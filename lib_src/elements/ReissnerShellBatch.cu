#include "ReissnerShellBatch.h"
#include "ReissnerShellBatchDiagnostics.h"
#include "ReissnerShellAssembly.h"

#include <array>
#include <cmath>
#include <cstring>
#include <new>

namespace tl::fea::reissner {
namespace {
using batch_detail::Control;
using batch_detail::Storage;
namespace sc = tlfea::contact;

ShellBatchReport Ok() { return {ShellBatchStatus::kSuccess, "OK"}; }

__global__ void MarkFailure(NodalAssemblyView view, std::uint32_t node) {
  RecordNodalAssemblyFailure(view, sc::Status::kInvalidArgument, node);
}

__device__ bool ValidateMass(Storage& storage, const NodalAssemblyView& view) {
  bool component_constraints = false;
  const auto n = storage.model.config.owner.node_count;
  for (std::size_t node = 0; node < n; ++node) {
    const unsigned bits = view.translation_fixed_bits[node];
    const unsigned rotation_fixed = view.rotation_fixed[node];
    const double inverse_mass = view.mass.inverse_mass[node], inverse_inertia = view.inverse_inertia[node];
    const double mass_product = inverse_mass * storage.model.nodal_mass[node];
    const double inertia_product = inverse_inertia * storage.model.nodal_inertia[node];
    if (bits > 7 || rotation_fixed > 1 || view.mass.fixed[node] != (bits == 7) ||
        !detail::Finite(inverse_mass) || !detail::Finite(inverse_inertia) ||
        (bits == 7 ? inverse_mass != 0 : (inverse_mass <= 0 || ::fabs(mass_product - 1) > 1e-12)) ||
        (rotation_fixed ? inverse_inertia != 0 : (inverse_inertia <= 0 || ::fabs(inertia_product - 1) > 1e-12))) {
      storage.control.status = ShellBatchStatus::kInvalidMass; storage.control.node = node; return false;
    }
    component_constraints |= bits != 0 && bits != 7;
  }
  const auto expected_model = component_constraints ? sc::TranslationMassModel::kUnspecified : sc::TranslationMassModel::kIsotropicLumped;
  if (view.mass.model != expected_model) { storage.control.status = ShellBatchStatus::kInvalidMass; return false; }
  return true;
}

__global__ void AssembleBatch(Storage* storage, NodalAssemblyView view) {
  auto& s = *storage;
  s.control = {}; s.base = {};
  s.control.diagnostics = batch_detail::BeginDiagnostics(s.model, view.accepted.base_epoch, view.attempt, ShellBatchPhase::kAcceptedBase);
  const auto n = s.model.config.owner.node_count;
  for (std::size_t i = 0; i < 6*n; ++i) s.force[i] = 0;
  if (view.result->base_epoch != view.accepted.base_epoch || view.result->attempt != view.attempt ||
      view.bounds->base_epoch != view.accepted.base_epoch || view.bounds->attempt != view.attempt ||
      !view.bounds->initialized || !view.bounds->valid || view.bounds->sealed || view.result->status != sc::Status::kOk) {
    s.control.status = ShellBatchStatus::kAssemblyFailure;
  } else if (ValidateMass(s, view)) {
    const DeviceNodalForceView own_force{s.force, s.force+n, s.force+2*n, s.force+3*n, s.force+4*n, s.force+5*n,
                                       n, view.accepted.base_epoch};
    for (unsigned e = 0; e < s.model.config.element_count; ++e) {
      if (!batch_detail::MeasureElement(s.model, e, view.accepted, s.element_result[e], s.control)) break;
      if (AccumulateShellForces(s.model.element[e].nodes, s.element_result[e], own_force) != ShellAssemblyStatus::kSuccess ||
          AccumulateShellForces(s.model.element[e].nodes, s.element_result[e], view.forces) != ShellAssemblyStatus::kSuccess) {
        s.control.status = ShellBatchStatus::kAssemblyFailure; s.control.element = e; break;
      }
    }
  }
  if (s.control.status != ShellBatchStatus::kSuccess) {
    RecordNodalAssemblyFailure(view, sc::Status::kInvalidArgument, s.control.node);
    return;
  }
  s.control.diagnostics.valid = true;
  s.base = s.control.diagnostics;
}

__global__ void EvaluatePrepared(Storage* storage, NodalPreparedView view) {
  auto& s = *storage;
  s.control = {};
  s.control.diagnostics = batch_detail::BeginDiagnostics(s.model, view.kinematics.base_epoch, view.attempt, ShellBatchPhase::kPreparedCandidate);
  if (!s.base.valid || s.base.owner_id != view.owner_id || s.base.base_epoch != view.kinematics.base_epoch ||
      s.base.attempt != view.attempt || s.base.configuration_id != s.model.config.configuration_id) {
    s.control.status = ShellBatchStatus::kStaleTrial; return;
  }
  for (unsigned e = 0; e < s.model.config.element_count; ++e)
    if (!batch_detail::MeasureElement(s.model, e, view.kinematics, s.element_result[e], s.control)) return;
  if (!batch_detail::MeasureInterval(s, view)) return;
  s.control.diagnostics.valid = true;
}

bool ValidKinematics(const DeviceNodalKinematicsView& view, const ReissnerShellBatchConfig& config) {
  return view.node_count == config.owner.node_count && view.position_xyz && view.velocity_xyz &&
         view.angular_velocity_xyz && view.orientation_wxyz && view.base_epoch >= config.owner.epoch;
}

bool SameIdentity(const ShellBatchDiagnostics& a, const ShellBatchDiagnostics& b) {
  return a.valid && b.valid && a.owner_id == b.owner_id && a.base_epoch == b.base_epoch &&
         a.attempt == b.attempt && a.configuration_id == b.configuration_id && a.phase == b.phase;
}

ShellBatchReport BuildModel(const ReissnerShellBatchConfig& config, const ReissnerShellBatchElement* elements,
                            batch_detail::Model& model) {
  if (!elements || !config.owner.owner_id || !config.owner.has_rotations || !config.configuration_id ||
      !std::isfinite(config.owner.fixed_dt) || config.owner.fixed_dt <= 0 || !std::isfinite(config.owner.time))
    return {ShellBatchStatus::kInvalidInput, "Batch requires an initialized rotational owner and immutable configuration identity"};
  if (!config.element_count || config.element_count > MaxReissnerShellBatchElements || !config.owner.node_count ||
      config.owner.node_count > MaxTranslationNodes || !config.max_device_bytes ||
      config.max_device_bytes > MaxReissnerShellBatchDeviceBytes || sizeof(Storage) > config.max_device_bytes)
    return {ShellBatchStatus::kResourceLimit, "Batch exceeds the admitted element/node/device budget"};
  if (config.drilling_policy != ShellDrillingInertiaPolicy::kEqualPhysicalTangential)
    return {ShellBatchStatus::kInvalidMass, "Batch requires explicitly declared equal physical/artificial drilling inertia"};
  model.config = config;
  bool seen[MaxTranslationNodes]{};
  Vec3 position[MaxTranslationNodes];
  Quaternion orientation[MaxTranslationNodes];
  for (unsigned e = 0; e < config.element_count; ++e) {
    model.element[e] = elements[e];
    const auto& element = model.element[e];
    if (ComputeShellMass(element.reference, element.section, config.drilling_policy, model.element_mass[e]) != ShellMassStatus::kSuccess)
      return {ShellBatchStatus::kInvalidMass, "Invalid shell mass input", e};
    ShellConfiguration initial;
    for (unsigned local = 0; local < 4; ++local) {
      const auto node = element.nodes[local];
      if (node >= config.owner.node_count) return {ShellBatchStatus::kInvalidInput, "Connectivity is outside owner node space", e};
      for (unsigned other = 0; other < local; ++other)
        if (element.nodes[other] == node) return {ShellBatchStatus::kInvalidInput, "Repeated Q4 node", e, static_cast<std::uint32_t>(node)};
      initial.position[local] = element.reference.initial_position[local];
      initial.rotation[local] = element.reference.initial_rotation[local];
      if (seen[node]) {
        const double position_error = batch_detail::Length(detail::Subtract(position[node], initial.position[local]));
        const double scale = batch_detail::Maximum(1, batch_detail::Length(position[node]));
        if (position_error > 1e-12 * scale || batch_detail::RotationAngle(orientation[node], initial.rotation[local]) > 1e-12)
          return {ShellBatchStatus::kInvalidInput, "Shared nodes have inconsistent immutable reference state", e, static_cast<std::uint32_t>(node)};
      } else {
        seen[node] = true; position[node] = initial.position[local]; orientation[node] = initial.rotation[local];
      }
      model.nodal_mass[node] += model.element_mass[e].node[local].mass;
      model.nodal_inertia[node] += model.element_mass[e].node[local].physical_tangential_inertia;
      if (!detail::Finite(model.nodal_mass[node]) || !detail::Finite(model.nodal_inertia[node]))
        return {ShellBatchStatus::kInvalidMass, "Assembled shell mass/inertia overflow", e, static_cast<std::uint32_t>(node)};
    }
    ShellResult result;
    const auto status = ComputeShellForce(element.reference, element.section, initial, result);
    if (status != ShellStatus::kSuccess) return {ShellBatchStatus::kElementFailure, "Invalid immutable shell setup", e, UINT32_MAX, status};
  }
  for (unsigned node = 0; node < config.owner.node_count; ++node)
    if (!seen[node]) return {ShellBatchStatus::kInvalidMass, "This batch must cover the owner's complete mass space", UINT32_MAX, node};
  return Ok();
}
}  // namespace

struct ReissnerShellBatch::Impl {
  Storage* device = nullptr;
  ReissnerShellBatchConfig config;
  Control control;
  std::array<ShellResult, MaxReissnerShellBatchElements> staging;
  std::uint64_t last_epoch = 0, last_attempt = 0;
  bool usable = true, has_base = false, has_results = false;
  cudaStream_t last_stream = nullptr;
  ~Impl() { if (device) cudaFree(device); }
  ShellBatchReport Check(cudaError_t error) {
    if (error == cudaSuccess) return Ok();
    usable = false; has_base = false; has_results = false;
    return {ShellBatchStatus::kDeviceFailure, cudaGetErrorString(error)};
  }
  ShellBatchReport ReadControl(cudaStream_t stream) {
    auto report = Check(cudaGetLastError()); if (report.status != ShellBatchStatus::kSuccess) return report;
    report = Check(cudaMemcpyAsync(&control, &device->control, sizeof(Control), cudaMemcpyDeviceToHost, stream));
    if (report.status != ShellBatchStatus::kSuccess) return report;
    report = Check(cudaStreamSynchronize(stream)); if (report.status != ShellBatchStatus::kSuccess) return report;
    return {control.status, control.status == ShellBatchStatus::kSuccess ? "OK" : "Shell batch validation failed",
            control.element, control.node, control.element_status};
  }
  ShellBatchReport FailAssembly(const NodalAssemblyView& view, ShellBatchReport report) {
    has_base = false; has_results = false;
    if (view.result && view.bounds) {
      MarkFailure<<<1,1,0,view.stream>>>(view, report.node);
      auto failure = Check(cudaGetLastError()); if (failure.status != ShellBatchStatus::kSuccess) return failure;
      failure = Check(cudaStreamSynchronize(view.stream)); if (failure.status != ShellBatchStatus::kSuccess) return failure;
    }
    return report;
  }
};

ReissnerShellBatch::ReissnerShellBatch() = default;
ReissnerShellBatch::~ReissnerShellBatch() = default;

ShellBatchReport ReissnerShellBatch::Initialize(const ReissnerShellBatchConfig& config,
                                               const ReissnerShellBatchElement* elements) {
  if (impl_) return {ShellBatchStatus::kInvalidInput, "Batch already initialized"};
  Storage initial;
  auto report = BuildModel(config, elements, initial.model);
  if (report.status != ShellBatchStatus::kSuccess) return report;
  try {
    auto candidate = std::make_unique<Impl>(); candidate->config = config;
    report = candidate->Check(cudaMalloc(reinterpret_cast<void**>(&candidate->device), sizeof(Storage)));
    if (report.status != ShellBatchStatus::kSuccess) return report;
    report = candidate->Check(cudaMemcpy(candidate->device, &initial, sizeof(Storage), cudaMemcpyHostToDevice));
    if (report.status != ShellBatchStatus::kSuccess) return report;
    impl_ = std::move(candidate); return Ok();
  } catch (const std::bad_alloc&) { return {ShellBatchStatus::kResourceLimit, "Host batch allocation failed"}; }
}

ShellBatchReport ReissnerShellBatch::Assemble(const NodalAssemblyView& view, ShellBatchDiagnostics* output) {
  if (!impl_) {
    Impl failure_channel;
    return failure_channel.FailAssembly(view, {ShellBatchStatus::kNotInitialized, "Batch is not initialized"});
  }
  auto& s = *impl_; s.has_base = false; s.has_results = false;
  if (!s.usable) return {ShellBatchStatus::kDeviceFailure, "CUDA batch is poisoned"};
  if (!output || !ValidKinematics(view.accepted, s.config) || !view.mass.inverse_mass || !view.mass.fixed ||
      !view.inverse_inertia || !view.translation_fixed_bits || !view.rotation_fixed || !view.bounds || !view.result ||
      view.forces.node_count != s.config.owner.node_count || view.mass.node_count != s.config.owner.node_count ||
      !view.forces.force_x || !view.forces.force_y || !view.forces.force_z || !view.forces.couple_x || !view.forces.couple_y || !view.forces.couple_z)
    return s.FailAssembly(view, {ShellBatchStatus::kInvalidInput, "Invalid owner assembly view or diagnostic output"});
  if (view.owner_id != s.config.owner.owner_id)
    return s.FailAssembly(view, {ShellBatchStatus::kWrongOwner, "Assembly belongs to another owner"});
  if (!view.attempt || view.accepted.base_epoch != view.forces.base_epoch || view.accepted.base_epoch != view.mass.base_epoch ||
      view.accepted.base_epoch < s.last_epoch || view.attempt <= s.last_attempt)
    return s.FailAssembly(view, {ShellBatchStatus::kStaleTrial, "Assembly is stale or this batch already contributed"});
  s.last_epoch = view.accepted.base_epoch; s.last_attempt = view.attempt; s.last_stream = view.stream;
  AssembleBatch<<<1,1,0,view.stream>>>(s.device, view);
  auto report = s.ReadControl(view.stream);
  if (report.status != ShellBatchStatus::kSuccess) return report;
  s.has_base = true; s.has_results = true;
  *output = s.control.diagnostics; return Ok();
}

ShellBatchReport ReissnerShellBatch::EvaluateCandidate(const NodalPreparedView& view, ShellBatchDiagnostics* output) {
  if (!impl_) return {ShellBatchStatus::kNotInitialized, "Batch is not initialized"};
  auto& s = *impl_; s.has_results = false;
  if (!s.usable) return {ShellBatchStatus::kDeviceFailure, "CUDA batch is poisoned"};
  if (!output || !ValidKinematics(view.kinematics, s.config) || !ValidKinematics(view.base_kinematics, s.config) ||
      !std::isfinite(view.proposed_time) || view.proposed_time <= s.config.owner.time)
    return {ShellBatchStatus::kInvalidInput, "Invalid prepared/base owner views or diagnostic output"};
  if (view.owner_id != s.config.owner.owner_id) return {ShellBatchStatus::kWrongOwner, "Candidate belongs to another owner"};
  if (!s.has_base || view.attempt != s.last_attempt || view.kinematics.base_epoch != s.last_epoch ||
      view.base_kinematics.base_epoch != s.last_epoch || view.stream != s.last_stream)
    return {ShellBatchStatus::kStaleTrial, "Candidate does not match this batch's completed assembly"};
  EvaluatePrepared<<<1,1,0,view.stream>>>(s.device, view);
  auto report = s.ReadControl(view.stream);
  if (report.status != ShellBatchStatus::kSuccess) return report;
  s.has_results = true;
  *output = s.control.diagnostics; return Ok();
}

ShellBatchReport ReissnerShellBatch::CopyElementResults(const ShellBatchDiagnostics& expected,
                                                       ShellResult* output, std::size_t capacity) {
  if (!impl_) return {ShellBatchStatus::kNotInitialized, "Batch is not initialized"};
  auto& s = *impl_;
  if (!s.usable) return {ShellBatchStatus::kDeviceFailure, "CUDA batch is poisoned"};
  if (!s.has_results || !SameIdentity(expected, s.control.diagnostics))
    return {ShellBatchStatus::kStaleTrial, "Result identity/phase does not match the latest successful diagnostic"};
  if (!output) return {ShellBatchStatus::kInvalidInput, "Missing element result output"};
  if (capacity < s.config.element_count) return {ShellBatchStatus::kResourceLimit, "Element result capacity is insufficient"};
  const std::size_t bytes = s.config.element_count * sizeof(ShellResult);
  const auto begin = reinterpret_cast<std::uintptr_t>(output), expected_begin = reinterpret_cast<std::uintptr_t>(&expected);
  if (bytes > UINTPTR_MAX - begin || sizeof(expected) > UINTPTR_MAX - expected_begin ||
      (begin < expected_begin + sizeof(expected) && expected_begin < begin + bytes))
    return {ShellBatchStatus::kInvalidInput, "Element output address range overflows or aliases expected identity"};
  auto report = s.Check(cudaGetLastError()); if (report.status != ShellBatchStatus::kSuccess) return report;
  report = s.Check(cudaMemcpyAsync(s.staging.data(), s.device->element_result, bytes, cudaMemcpyDeviceToHost, s.last_stream));
  if (report.status != ShellBatchStatus::kSuccess) return report;
  report = s.Check(cudaStreamSynchronize(s.last_stream)); if (report.status != ShellBatchStatus::kSuccess) return report;
  std::memcpy(output, s.staging.data(), bytes); return Ok();
}

NodalAllocationInfo ReissnerShellBatch::allocations() const noexcept {
  return impl_ ? NodalAllocationInfo{sizeof(Storage), 1} : NodalAllocationInfo{};
}

}  // namespace tl::fea::reissner
