#pragma once

#include "FENodalState.h"
#include <array>

namespace tl::fea {
namespace nodal_detail {
enum class Phase { Idle, Assembling, Sealed, Ready };
struct Control {
  stability::RowBounds rows;
  NodalAssemblyResult assembly;
  stability::StepLimit limit;
  NodalStatus status = NodalStatus::Ok;
  std::uint32_t node = UINT32_MAX;
};
}  // namespace nodal_detail

// Private runtime storage, shared only with the non-owning advance operation.
struct FENodalState::Impl {
  ~Impl();
  NodalReport Check(cudaError_t);
  NodalReport SynchronizeControl();
  NodalReport Reject(NodalStatus, const char*, std::uint32_t = UINT32_MAX);
  bool Matches(std::uint64_t owner, std::uint64_t epoch, std::uint64_t trial) const;
  NodalStateConfig config;
  NodalStamp stamp;
  NodalAllocationInfo allocation;
  std::uint64_t attempt = 0;
  double candidate_time = 0;
  bool usable = true;
  nodal_detail::Phase phase = nodal_detail::Phase::Idle;
  cudaStream_t stream = nullptr;
  double *accepted = nullptr, *trial = nullptr, *scratch = nullptr, *inverse = nullptr;
  std::uint8_t* fixed = nullptr;
  nodal_detail::Control* control = nullptr;
  nodal_detail::Control host_control;
  // AoS accepted x/v readback. No allocation or host vector growth after startup.
  std::array<double, 6 * MaxTranslationNodes> staging{};
};
}  // namespace tl::fea
