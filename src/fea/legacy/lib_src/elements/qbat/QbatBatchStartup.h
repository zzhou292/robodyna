// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "QbatBatchArena.h"
#include "../ShellFormulationScope.h"

namespace tl::fea::qbat::batch_detail {
BatchReport ValidateStartup(const BatchConfig&,const ShellFormulationScope&) noexcept;
BatchReport BuildStartup(const BatchConfig&,const ShellFormulationScope&,Storage&,BatchDiagnostics&) noexcept;
BatchReport BuildElements(const BatchConfig&,const ShellFormulationScope&,Storage&,BatchDiagnostics&) noexcept;
// Called only after all host parameters have been validated, immediately before
// copying the staged arena. No curve ownership or pointer arithmetic is guessed.
bool RebaseMaterials(Storage& host,const Storage& device,
    const ShellBatchPlasticityBinding&) noexcept;
} // namespace tl::fea::qbat::batch_detail
