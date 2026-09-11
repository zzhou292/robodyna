// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../QbatBatchStartup.h"
#include "../../../assembly/ShellPhysicalBinding.h"
#include "../../ShellPhysicalOwner.h"

namespace tl::fea::qbat::mapped {
struct Forecast {
  batch_detail::Layout device;
  shell_physical_owner::ProofLayout proof;
  std::size_t host_bytes=0;
};
BatchReport Validate(const BatchConfig&,const ShellPhysicalBinding&,
    const NodalCinWitnessSource&) noexcept;
BatchReport MakeForecast(const BatchConfig&,const ShellPhysicalBinding&,
    const NodalCinWitnessSource&,std::size_t private_bytes,Forecast&) noexcept;
BatchReport BuildModel(const BatchConfig&,const ShellPhysicalBinding&,
    batch_detail::Storage&,BatchDiagnostics&) noexcept;
} // namespace tl::fea::qbat::mapped
