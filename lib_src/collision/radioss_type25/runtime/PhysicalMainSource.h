// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Types.h"
#include "../startup/PostGapmTypes.h"
#include "lib_src/assembly/ShellPhysicalBinding.h"
namespace tlfea::contact::radioss_type25::runtime_detail {
// Incidence/source identity only: no native support winner, coefficient,
// normal, activity, source-history or runtime-ready authority is produced.
// The source caller keeps every borrowed descriptor immutable during this call.
struct PhysicalMainValidation {
  TransactionReport report;
  std::size_t startup_host_bytes=0;
};
PhysicalMainValidation ValidateMixedPhysicalMains(const tl::fea::ShellPhysicalBinding&,
    const startup::MixedSidesSnapshot&,const startup::PostGapmTopology&,
    std::size_t max_host_bytes) noexcept;
PhysicalMainValidation ValidateMixedPhysicalMains(const tl::fea::ShellPhysicalBinding&,
    const startup::Snapshot&,std::size_t max_host_bytes) noexcept;
} // namespace tlfea::contact::radioss_type25::runtime_detail
