// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Batch.h"
#include "State.h"
#include "../../ShellPhysicalOwner.h"

namespace tl::fea::beam18::batch_detail {
struct Control {
  BatchStatus status = BatchStatus::Success;
  std::size_t parent = SIZE_MAX, node = SIZE_MAX;
  int element_status = 0;
  BatchDiagnostics diagnostics;
};
struct Storage {
  BatchConfig config;
  std::uint64_t source_instance_id = 0;
  Parent* parents = nullptr;
  Material* materials = nullptr;
  State* slab[2]{};
  int* status = nullptr;
  std::size_t count = 0;
  Control control;
};
struct ArenaLayout {
  util::ArenaRegion header, parents, materials, curves, slab[2], status, staging;
  std::size_t bytes = 0, staging_bytes = 0, curve_points = 0;
  shell_physical_owner::ProofLayout proof;
};
struct Counts { std::size_t parents = 0, materials = 0, curve_points = 0; };
bool MakeLayout(Counts, const BatchConfig&, ArenaLayout&) noexcept;
BatchReport Plan(const BatchConfig&, const Model&, ArenaLayout&) noexcept;
Storage RebasedHeader(void*, const ArenaLayout&) noexcept;
BatchReport BuildUpload(const BatchConfig&, const Model&, util::HostArena&, const ArenaLayout&, Storage&);
Material ExpectedMaterial(const Model&, std::size_t, double*) noexcept;
void RebaseCurves(const Model&, const ArenaLayout&, void*, Storage&) noexcept;
} // namespace tl::fea::beam18::batch_detail
