// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../NodalWallMappedContact.h"
#include "lib_src/assembly/ShellPhysicalBinding.h"
#include "../NodalWallContactArena.h"
#include "Layout.h"
#include "lib_src/elements/ShellPhysicalOwner.h"
#include "lib_utils/BoundedStartupArray.h"

namespace tlfea::contact::nodal_wall_mapped {
using Code=NodalWallDeviceStatus;
namespace fe=tl::fea;
namespace d=nodal_wall_device_detail;
struct Parent {
  fe::ShellBindingFamily family=fe::ShellBindingFamily::None;
  std::uint32_t index=0;
  std::uint64_t source_id=0;
};
NodalWallDeviceReport Preflight(const NodalWallDeviceConfig&,const NodalWallWeights&,
    const NodalWallMappedSource&,NodalWallMappedLimits,std::size_t,d::ArenaLayout&,Layout&,
    fe::shell_physical_owner::ProofLayout&,fe::ShellMappedFootprint&) noexcept;
bool SameCertificate(Q4CertifiedIntegral,Q4CertifiedIntegral) noexcept;
bool SameDiagnostics(const NodalWallMappedDiagnostics&,const NodalWallMappedDiagnostics&) noexcept;
} // namespace tlfea::contact::nodal_wall_mapped

namespace tlfea::contact {
struct NodalWallMappedContact::Impl {
  Impl(const NodalWallMappedSource&);
  ~Impl();
  Impl(const Impl&)=delete;
  Impl& operator=(const Impl&)=delete;
  tl::fea::ShellPhysicalBinding physical;
  tl::fea::NodalRigidAssemblyBinding rigid;
  const tl::fea::ShellBatchPublication* publication=nullptr;
  tl::fea::ShellPhysicalParticipants participants;
  tl::fea::FENodalState* owner=nullptr;
  tl::fea::ShellPhysicalPublicationIdentity identity;
  NodalWallDeviceConfig config;
  tl::fea::ShellMappedFootprint forecast;
  nodal_wall_device_detail::PreparedModel prepared;
  nodal_wall_device_detail::Storage shadow;
  nodal_wall_device_detail::Storage* device=nullptr;
  nodal_wall_mapped::Layout layout;
  tl::util::HostArena host;
  void* device_sidecar=nullptr;
  nodal_wall_mapped::Sidecar local,remote;
  tl::util::BoundedStartupArray<nodal_wall_mapped::Parent,0> parents;
  tl::util::BoundedStartupArray<std::uint8_t,0> activity;
  tl::util::BoundedStartupArray<tl::fea::NodalRigidGroupSnapshot,0> snapshots;
  std::size_t witnesses=0,accepted_active=0,proposed_active=0;
  nodal_wall_device_detail::Control control;
  NodalWallMappedDiagnostics available;
  tl::fea::NodalAssemblyView base;
  tl::fea::NodalStamp base_stamp;
  cudaStream_t stream=nullptr;
  std::uint64_t last_attempt=0,last_candidate=0;
  bool usable=true,has_base=false,has_results=false;
  NodalWallDeviceReport Check(cudaError_t);
  NodalWallDeviceReport ReadControl();
  NodalWallDeviceReport ReadDiagnostics(bool,NodalWallMappedDiagnostics&);
  NodalWallDeviceReport ReadResults(const NodalWallMappedDiagnostics&);
  NodalWallDeviceReport CaptureActivity(const tl::fea::NodalTrialToken*,
      const tl::fea::ShellPhysicalDiagnostics*);
  NodalWallDeviceReport CaptureBodies();
  NodalWallDeviceReport PrepareSources(const NodalWallWeights&,const tl::fea::NodalCinWitnessSource&,
      const double* positions);
  bool OutputDisjoint(const void*,std::size_t) const noexcept;
  NodalWallDeviceReport Authenticate(tl::fea::FENodalState&) const noexcept;
};
} // namespace tlfea::contact
