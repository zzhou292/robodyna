// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../Type25BatchArena.h"
#include "../../../assembly/ShellPhysicalBinding.h"
#include "../../ShellPhysicalOwner.h"

namespace tl::fea::type25::mapped {
struct Forecast {
  batch_detail::ArenaLayout device;
  shell_physical_owner::ProofLayout proof;
  std::size_t host_bytes=0;
};
BatchReport MakeForecast(const BatchConfig&,const ShellPhysicalBinding&,
    const NodalCinWitnessSource&,CapacityProfile,std::size_t private_bytes,Forecast&) noexcept;
BatchReport ValidateModel(const Model&) noexcept;
BatchReport BuildModel(const BatchConfig&,const ShellPhysicalBinding&,util::HostArena&,
    const batch_detail::ArenaLayout&,batch_detail::Storage&,BatchDiagnostics&);
} // namespace tl::fea::type25::mapped
