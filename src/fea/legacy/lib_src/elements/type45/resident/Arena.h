// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "State.h"
#include "../../ShellPhysicalOwner.h"
#include "lib_src/solvers/NodalCinPhysicalMains.h"

namespace tl::fea::type45::resident_detail {
struct Control {
  BatchStatus status=BatchStatus::Success;
  Status joint_status=Status::Success;
  std::size_t joint=SIZE_MAX,node=SIZE_MAX;
  BatchDiagnostics diagnostics;
};
struct Storage {
  BatchConfig config;
  std::uint64_t source_instance_id=0;
  Joint* joints=nullptr;
  State* slab[2]{};
  AutomaticStiffnessContext* contexts=nullptr;
  Status* status=nullptr;
  std::size_t count=0;
  Control control;
};
struct ArenaLayout {
  util::ArenaRegion header,joints,slab[2],contexts,status;
  util::ArenaRegion staging,host_contexts,mains;
  std::size_t bytes=0,staging_bytes=0;
  shell_physical_owner::ProofLayout proof;
};
bool MakeLayout(std::size_t joints,const BatchConfig&,ArenaLayout&) noexcept;
BatchReport Plan(const BatchConfig&,const Model&,ArenaLayout&) noexcept;
Storage RebasedHeader(void*,const ArenaLayout&) noexcept;
BatchReport BuildUpload(const BatchConfig&,const Model&,util::HostArena&,const ArenaLayout&);
bool SameDiagnostics(const BatchDiagnostics&,const BatchDiagnostics&) noexcept;
bool SameConfig(const BatchConfig&,const BatchConfig&) noexcept;
bool ModelOutputDisjoint(const Model&,const void*,std::size_t) noexcept;
void LaunchInitialize(Storage*,cudaStream_t);
void LaunchCandidate(Storage*,unsigned accepted,unsigned trial,NodalPreparedView,BatchDiagnostics);
void LaunchAssembly(Storage*,unsigned accepted,NodalAssemblyView,NodalCinAssemblyView);
} // namespace tl::fea::type45::resident_detail
