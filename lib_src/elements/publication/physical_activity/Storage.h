// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../PhysicalActivitySnapshot.h"
#include "Device.h"
#include "../../ShellBatchPublication.h"
#include "lib_utils/BoundedArena.h"
namespace tl::fea::physical_activity {
enum class Phase : std::uint8_t { Idle, Accepted, Prepared, Exhausted };
struct Layout {
  util::ArenaRegion masks[3], laws, control;
  std::size_t bytes = 0;
};
bool KinematicsOutputDisjoint(const DeviceNodalKinematicsView&, const void*, std::size_t) noexcept;
bool MakeLayout(std::size_t parents, std::size_t cap, Layout&) noexcept;
PhysicalActivityReport GuardOtherFamilies(const ShellPhysicalBinding&, const ShellPhysicalDiagnostics&) noexcept;
struct State {
  explicit State(const ShellPhysicalBinding& p) noexcept : physical(p) {}
  ~State();
  ShellPhysicalBinding physical;
  FENodalState* owner = nullptr;
  ShellBatchPublication* publication = nullptr;
  ShellPhysicalParticipants participants;
  ShellPhysicalPublicationIdentity identity;
  PhysicalActivityForecast forecast;
  Layout layout;
  void* arena = nullptr;
  std::uint8_t *base = nullptr, *current = nullptr, *staging = nullptr, *laws = nullptr;
  FamilyControl* device_control = nullptr;
  FamilyControl host_control[2];
  PhysicalActivityFamilySummary summaries[2];
  NodalStamp stamp;
  NodalTrialToken token;
  NodalAssemblyView assembly;
  NodalPreparedView prepared;
  ShellPhysicalDiagnostics diagnostics;
  cudaStream_t stream = nullptr;
  std::uint64_t generation = 0, attempt = 0;
  Phase phase = Phase::Idle;
  bool usable = true;
  void Invalidate() noexcept;
  bool AdvanceGeneration() noexcept;
  bool OutputDisjoint(const void*, std::size_t) const noexcept;
  PhysicalActivityReport Sources() const noexcept;
  bool Live(Phase) const noexcept;
  bool Authenticates(const PhysicalAcceptedActivityReceipt&) const noexcept;
  bool Authenticates(const PhysicalPreparedActivityReceipt&) const noexcept;
  PhysicalActivityDeviceView View() const noexcept;
  PhysicalActivityReport Capture(const NodalTrialToken&, const NodalAssemblyView*,
      const NodalPreparedView*, const ShellPhysicalDiagnostics&) noexcept;
};
PhysicalActivityReport PublicationReport(const ShellPublicationReport&) noexcept;
} // namespace tl::fea::physical_activity
