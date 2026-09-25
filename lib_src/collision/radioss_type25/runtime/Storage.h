// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "lib_src/collision/RadiossType25Transaction.h"
#include "lib_src/collision/RadiossType25Candidates.h"
#include "lib_src/collision/RadiossType25Search.h"
#include "lib_src/collision/RadiossType25AssemblyDevice.h"
#include "Launch.h"
#include "Source.h"
namespace tlfea::contact::radioss_type25 {
struct Transaction::Impl {
  explicit Impl(const tl::fea::ShellPhysicalBinding& value):physical(value){}
  ~Impl();
  tl::fea::ShellPhysicalBinding physical;
  tl::fea::ShellPhysicalParticipants participants;
  tl::fea::ShellPhysicalPublicationIdentity identity;
  tl::fea::FENodalState* owner=nullptr;
  tl::fea::ShellBatchPublication* publication=nullptr;
  tl::fea::ShellPhysicalScratchParticipation issuer;
  tl::fea::NativeContactPublicationState state; // Destroy before issuer.
  candidates::Inventory inventory[2];search::Maintenance maintenance[2];
  assembly::DeviceIncidenceBuilder incidence;
  candidates::InventoryView inventory_view[2];
  TransactionConfig config;FixedMainSource source;TransactionLimits limits;
  TransactionForecast forecast;TransactionDiagnostics diagnostics;
  runtime_detail::Layout layout;runtime_detail::Device device;runtime_detail::Control control;
  units_detail::Factors units{};void* arena=nullptr;cudaStream_t stream=nullptr;
  tl::util::HostArena readback;tl::util::ArenaRegion readback_rows,readback_secondary;
  tl::fea::NodalAssemblyView assembly_view;tl::fea::NodalStamp assembly_stamp;
  tl::fea::NativeContactSelectors trial_selectors;
  enum class Phase { Idle,Assembled,Sealed };Phase phase=Phase::Idle;
  bool usable=true;
  TransactionReport Fence(cudaError_t) noexcept;
  TransactionReport Fail(TransactionReport) noexcept;
  void DiscardLocal() noexcept;
  bool OutputDisjoint(const void*,std::size_t) const noexcept;
};
} // namespace tlfea::contact::radioss_type25
