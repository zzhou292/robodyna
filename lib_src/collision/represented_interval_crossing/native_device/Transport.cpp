// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Workspace.h"

namespace tlfea::contact::represented_interval_crossing::native_device {
void Workspace::FaultScope(const AuthenticatedWork& work) noexcept {
  if (work.scene_.cohort_) {
    report_.fault_cohort_begin = work.scene_.cohort_->begin_;
    report_.fault_cohort_count = work.scene_.cohort_->count_;
  } else {
    report_.fault_cohort_begin = work.first_ordinal_;
    report_.fault_cohort_count = work.pair_count();
  }
}
RepresentedIntervalReport Workspace::InvalidPublication(const AuthenticatedWork& work,
    const char* message, std::size_t ordinal) noexcept {
  usable_ = false;
  report_.status = RepresentedIntervalDeviceStatus::DeviceFailure;
  report_.message = message;
  FaultScope(work);
  RepresentedIntervalReport failure;
  failure.status = RepresentedIntervalStatus::ResourceLimit;
  failure.message = message;
  if (ordinal != SIZE_MAX) {
    if (work.scene_.cohort_) {
      report_.fault_pair_ordinal = ordinal;
      if (ordinal >= work.first_ordinal_ && ordinal - work.first_ordinal_ < work.pair_count())
        failure.input_pair = work.pairs()[ordinal - work.first_ordinal_].input_pair;
    } else {
      report_.fault_pair_ordinal = work.first_ordinal_ + ordinal;
      failure.input_pair = ordinal;
    }
  }
  return failure;
}
RepresentedIntervalReport Workspace::Submit(const AuthenticatedWork& work,
    std::size_t jobs) noexcept {
  if (!jobs) return {};
  auto error = cudaSuccess;
  if (!work.scene_.uploaded_) {
    error = cudaMemcpyAsync(tl::util::ArenaPointer<RepresentedTrianglePath>(device_, layout_.paths),
        work.paths(), work.path_count() * sizeof(*work.paths()), cudaMemcpyHostToDevice, stream_);
    if (error == cudaSuccess) ++report_.scene_uploads;
  }
  if (error == cudaSuccess)
    error = cudaMemcpyAsync(tl::util::ArenaPointer<DeviceJob>(device_, layout_.jobs),
        jobs_, jobs * sizeof(*jobs_), cudaMemcpyHostToDevice, stream_);
  if (error == cudaSuccess) {
    ++report_.batches;
    error = Launch(device_, layout_, work.path_count(), jobs, work.limits(), stream_);
  }
  if (error == cudaSuccess)
    error = cudaMemcpyAsync(results_, tl::util::ArenaPointer<DeviceResult>(device_, layout_.results),
        jobs * sizeof(*results_), cudaMemcpyDeviceToHost, stream_);
  // Drain every enqueued borrowed read, including when a later copy failed.
  // Keep the first CUDA error and preserve the pre-cohort native publication.
  const auto synchronized = cudaStreamSynchronize(stream_);
  if (error == cudaSuccess) error = synchronized;
  if (error != cudaSuccess) {
    FaultScope(work);
    return DeviceFailure(error);
  }
  for (std::size_t i = 0; i < jobs; ++i)
    if (results_[i].execution != PairExecution::Complete)
      return InvalidPublication(work,
          "Native CUDA input/domain validation disagreed with authenticated host work",
          jobs_[i].pair.input_pair);
  // Numerical Unresolved/ExactArithmeticRange is a complete typed result, not
  // a CUDA failure. The unchanged native fold and physical policy decide it.
  work.scene_.uploaded_ = true;
  return {};
}
}  // namespace tlfea::contact::represented_interval_crossing::native_device
