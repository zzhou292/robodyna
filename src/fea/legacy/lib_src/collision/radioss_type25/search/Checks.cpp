// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Storage.h"
#include "Launch.h"
#include "Ranges.h"
#include "lib_src/solvers/NodalTrialIdentity.h"
namespace tlfea::contact::radioss_type25::search {
namespace {
bool Empty(VectorView view) noexcept {
  return !view.data && !view.node_count && !view.node_stride && !view.component_stride;
}
bool Separate(const void* a, std::size_t bytes, const void* b,
    std::size_t extent) noexcept {
  return !bytes || !extent || tl::fea::trial_identity::Disjoint(a, bytes, b, extent);
}
bool Field(VectorView view, std::size_t nodes, const void* arena,
    std::size_t arena_bytes) noexcept {
  std::size_t bytes = 0;
  return detail::VectorSpan(view, nodes, bytes) &&
      Separate(view.data, bytes, arena, arena_bytes);
}
}
bool Maintenance::Impl::OutputDisjoint(const void* output, std::size_t bytes,
    const Current& input) const noexcept {
  // Check() admitted these descriptors before this helper is reached. Include
  // the entire strided extent, so padding cannot be overwritten by publication.
  std::size_t positions = 0, velocities = 0;
  if (!detail::VectorSpan(input.positions, source.physical_nodes, positions))
    return false;
  if (!Empty(input.velocities) &&
      !detail::VectorSpan(input.velocities, source.physical_nodes, velocities))
    return false;
  return output && Separate(output, bytes, this, sizeof(*this)) &&
      Separate(output, bytes, arena, layout.forecast.device_bytes) &&
      Separate(output, bytes, &input, sizeof(input)) &&
      Separate(output, bytes, input.positions.data, positions) &&
      Separate(output, bytes, input.velocities.data, velocities) &&
      Separate(output, bytes, input.secondary_stiffness,
          input.secondary_count * sizeof(double)) &&
      Separate(output, bytes, input.main_gaps, input.main_gap_count * sizeof(double)) &&
      Separate(output, bytes, input.main_node_activity, input.main_node_activity_count);
}
Status Maintenance::Impl::Check(const Current& input, bool capture) const noexcept {
  if (!usable) return Status::Unusable;
  if (!detail::Same(input.stamp.source, source.stamp))
    return Status::UnsupportedLifecycle;
  if (!input.stamp.attempt || (reference && input.stamp.epoch < accepted_stamp.epoch))
    return Status::StaleReference;
  const auto nodes = source.physical_nodes;
  const auto bytes = layout.forecast.device_bytes;
  if (!Field(input.positions, nodes, arena, bytes)) return Status::InvalidInput;
  if (capture) {
    if (!Empty(input.velocities) && !Field(input.velocities, nodes, arena, bytes))
      return Status::InvalidInput;
  } else if (!Field(input.velocities, nodes, arena, bytes)) {
    return Status::InvalidInput;
  }
  if (input.secondary_count != source.secondaries ||
      !detail::Span(input.secondary_stiffness, input.secondary_count) ||
      !Separate(input.secondary_stiffness, input.secondary_count * sizeof(double), arena, bytes))
    return Status::InvalidInput;
  if (source.activity_policy == ActivityPolicy::MonotoneRetirement) {
    if (input.main_node_activity_count != nodes ||
        !detail::Span(input.main_node_activity, nodes) ||
        !Separate(input.main_node_activity, nodes, arena, bytes)) return Status::InvalidInput;
  } else if (input.main_node_activity || input.main_node_activity_count) return Status::InvalidInput;
  const auto gaps = layout.forecast.reference_gaps;
  if (input.main_gap_count != gaps || !detail::Span(input.main_gaps, gaps) ||
      !Separate(input.main_gaps, gaps * sizeof(double), arena, bytes))
    return Status::InvalidInput;
  return Status::Ok;
}
Status Maintenance::Impl::Result(Status status, const Current* input,
    std::size_t row) noexcept {
  failure = {};
  failure.status = status;
  if (status != Status::Ok && input) {
    failure.stamp = input->stamp;
    failure.query_available = true;
    failure.input_row = row;
    failure.row_available = row != SIZE_MAX;
  }
  return status;
}
Status Maintenance::Impl::Execute(const Current& input, unsigned slab, bool capture,
    double previous_dt, bool force_sort, ReferenceCapturePolicy policy) noexcept {
  auto error = cudaGetLastError();
  if (error == cudaSuccess) {
    error = detail::Run(device, input, slab, capture, source.margin, previous_dt,
        force_sort, reference, policy, stream);
  }
  if (error == cudaSuccess) {
    error = cudaMemcpyAsync(&control, device.control, sizeof(control),
        cudaMemcpyDeviceToHost, stream);
  }
  // Drain borrowed input/readback lifetimes even after enqueue/launch failure.
  const auto drained = cudaStreamSynchronize(stream);
  if (error != cudaSuccess || drained != cudaSuccess) {
    usable = false;
    pending = false;
    return Result(Status::DeviceFailure, &input);
  }
  return Result(control.partial.status, &input, control.partial.invalid);
}
} // namespace tlfea::contact::radioss_type25::search
