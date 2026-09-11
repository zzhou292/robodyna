// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../Storage.h"
#include "../../../ShellPhysicalOwner.h"

namespace tl::fea::type13::mapped {
struct Forecast {
  batch_detail::ArenaLayout device;
  shell_physical_owner::ProofLayout proof;
  std::size_t host_bytes = 0;
};
BatchReport MakeForecast(const BatchConfig&, const ShellPhysicalBinding&,
    const NodalRigidAssemblyBinding&, const NodalCinWitnessSource&,
    BatchMappedLimits, std::size_t private_bytes, Forecast&) noexcept;
} // namespace tl::fea::type13::mapped
