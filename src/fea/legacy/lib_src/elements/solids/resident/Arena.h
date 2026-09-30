// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Batch.h"
#include "MeasurementTypes.h"
#include "ExtendedScratch.h"
#include "AssemblyTypes.h"
#include "controlled/Storage.h"
#include "../../ShellPhysicalOwner.h"

namespace tl::fea::solids::batch_detail {
template<class Traits> struct DeviceFamily {
  typename Traits::Parent* parents = nullptr;
  State<Traits>* slab[2]{};
  int* status = nullptr;
  std::size_t count = 0;
  std::uint8_t* result_valid = nullptr;
  MeasurementOperands<Traits::nodes>* measurement = nullptr;
};
struct Control {
  BatchStatus status = BatchStatus::Success;
  Family family = Family::Solid18;
  std::size_t parent = SIZE_MAX, node = SIZE_MAX;
  int element_status = 0;
  BatchDiagnostics diagnostics;
};
struct Storage {
  BatchConfig config;
  std::uint64_t source_instance_id = 0;
  solid18::Material* material36 = nullptr;
  solid24::Material* material42 = nullptr;
  DeviceFamily<Traits18> solid18;
  Scratch18* scratch18 = nullptr;
  DeviceFamily<Traits24> solid24;
  DeviceFamily<Traits6z> solid6z;
  DeviceFamily<Traits18Law44> solid18_law44;
  DeviceFamily<Traits18Law90> solid18_law90;
  solid18::law44::Material* material44 = nullptr;
  solid18::total_strain::Material* material90 = nullptr;
  ExtendedScratch<Traits18Law44>* scratch44 = nullptr;
  ExtendedScratch<Traits18Law90>* scratch90 = nullptr;
  AssemblyMemory assembly;
  Control control;
  controlled::Storage controlled;
};
struct FamilyLayout {
  util::ArenaRegion parents, slab[2], status, staging, result_valid, measurement;
};
struct ArenaLayout {
  controlled::Layout controlled;
  util::ArenaRegion header, material36, material42, curves, scratch18;
  FamilyLayout solid18, solid24, solid6z, solid18_law44, solid18_law90;
  util::ArenaRegion material44, material90, scratch44, scratch90;
  util::ArenaRegion assembly_offsets, assembly_incidence, assembly_nodes;
  std::size_t bytes = 0, staging_bytes = 0, curve_points = 0;
  shell_physical_owner::ProofLayout proof;
};
struct Counts {
  std::size_t solid18 = 0, solid24 = 0, solid6z = 0;
  std::size_t material36 = 0, material42 = 0, curve_points = 0;
  std::size_t solid18_law44 = 0, solid18_law90 = 0, material44 = 0, material90 = 0;
  std::size_t analytic_material44 = 0;
  controlled::Counts controlled;
};
bool MakeLayout(Counts, const BatchConfig&, ArenaLayout&) noexcept;
BatchReport Plan(const BatchConfig&, const Model&, ArenaLayout&) noexcept;
Storage RebasedHeader(void*, const ArenaLayout&) noexcept;
BatchReport BuildUpload(const BatchConfig&, const Model&, util::HostArena&,
    const ArenaLayout&, Storage&);
// Rebase the device material curve pointers once, before upload. No History
// containing a host curve pointer is copied to the device.
BatchReport RebaseCurves(const Model&, const ArenaLayout&, void* device,
    Storage& host_header) noexcept;
} // namespace tl::fea::solids::batch_detail
