// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "NodalCinLayout.h"
#include <vector>

namespace tl::fea::nodal_detail {
struct CinStorage {
  CinStorage() = default;
  ~CinStorage();
  CinStorage(const CinStorage&) = delete;
  CinStorage& operator=(const CinStorage&) = delete;
  constraints::tied_shell::TiedCinAttachmentModel source;
  std::vector<constraints::tied_shell::cin::StageRow> rows;
  std::vector<constraints::tied_shell::cin::ActiveWitness> witnesses;
  std::vector<std::uint8_t> dependent;
  std::vector<std::uint32_t> first_witness;
  CinLayout layout;
  std::size_t state_offset = 0;
  std::uint64_t qualification_id = 0;
  void* arena = nullptr;
  constraints::tied_shell::cin::StageView device;
  std::uint8_t* activity = nullptr;
  constraints::tied_shell::Patch* patches = nullptr;
  double* work = nullptr;
  cin_advance::FailureKey* failure = nullptr;
  cin_advance::FailureKey* input_failure = nullptr;
  cin_advance::screen::Summary* screen = nullptr;
  cin_advance::groups::Report* group_reports = nullptr;
  constraints::tied_shell::cin::detail::PreparedForceRow* prepared_transfers = nullptr;
  cin_advance::recovery::Row* prepared_recovery = nullptr;
  cin_advance::recovery::FailureRow* recovery_failure = nullptr;
  cudaError_t Upload(cudaStream_t);
  cudaError_t ResetTrial(cudaStream_t);
  void InitializeState(double* state, const NodalCinStartup&, const double* inverse_mass, const NodalDofConfig&) const noexcept;
};
NodalReport ForecastCinStorage(const NodalCinStartup&, const NodalStateConfig&, CinLayout&, std::size_t group_count = 0) noexcept;
NodalReport PrepareCinStorage(const NodalCinStartup&, const NodalStateConfig&,
    HostNodalKinematicsView, const double*, const NodalDofConfig&,
    const NodalRigidGroupModel*, const CinLayout&, std::unique_ptr<CinStorage>&,
    const NodalRigidAssemblyBinding* = nullptr);
} // namespace tl::fea::nodal_detail
