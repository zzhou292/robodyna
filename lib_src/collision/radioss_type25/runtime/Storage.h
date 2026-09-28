// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "lib_src/collision/RadiossType25Transaction.h"
#include "lib_src/collision/RadiossType25Candidates.h"
#include "lib_src/collision/RadiossType25Search.h"
#include "lib_src/collision/RadiossType25AssemblyDevice.h"
#include "Launch.h"
#include "Source.h"
#include "ActivityRuntime.h"
#include "lib_src/elements/publication/PhysicalActivePrefix.h"
namespace tlfea::contact::radioss_type25 {
struct Transaction::Impl {
  explicit Impl(const tl::fea::ShellPhysicalBinding& value):physical(value){}
  ~Impl();
  tl::fea::ShellPhysicalBinding physical;
  tl::fea::ShellPhysicalParticipants participants;
  tl::fea::ShellPhysicalPublicationIdentity identity;
  tl::fea::PhysicalActivePrefix active_prefix;
  std::unique_ptr<runtime_detail::ActivityRuntime> activity;
  tl::fea::FENodalState* owner=nullptr;
  tl::fea::ShellBatchPublication* publication=nullptr;
  tl::fea::ShellPhysicalScratchParticipation issuer;
  tl::fea::NativeContactPublicationState state; // Destroy before issuer.
  candidates::Inventory inventory[2];search::Maintenance maintenance[2];
  assembly::DeviceIncidenceBuilder incidence;
  candidates::InventoryView inventory_view[2];
  TransactionConfig config;ContactSourceInput source;TransactionLimits limits;
  TransactionForecast forecast;TransactionDiagnostics diagnostics;
  TransactionInitializationDiagnostics initialization;
  runtime_detail::Layout layout;runtime_detail::Device device;runtime_detail::Control control;
  units_detail::Factors units{};void* arena=nullptr;cudaStream_t stream=nullptr;
  tl::util::HostArena readback;tl::util::ArenaRegion readback_rows,readback_secondary;
  tl::fea::NodalAssemblyView assembly_view;tl::fea::NodalStamp assembly_stamp;
  tl::fea::NativeContactSelectors trial_selectors;
  enum class Phase { Idle,Assembled,Sealed };Phase phase=Phase::Idle;
  bool usable=true,normal_ready=false;
  TransactionReport CaptureAcceptedActivity(const tl::fea::NodalTrialToken&,
      const tl::fea::NodalAssemblyView&,const tl::fea::NativeContactPublicationSnapshot&) noexcept;
  TransactionReport StageCandidateActivity(const tl::fea::NodalTrialToken&,
      const tl::fea::NodalPreparedView&,const tl::fea::ShellPhysicalDiagnostics&) noexcept;
  TransactionReport SelectActivity(unsigned) noexcept;
  TransactionReport Fence(cudaError_t) noexcept;
  TransactionReport Fail(TransactionReport) noexcept;
  void DiscardLocal() noexcept;
  bool OutputDisjoint(const void*,std::size_t) const noexcept;
};
} // namespace tlfea::contact::radioss_type25
