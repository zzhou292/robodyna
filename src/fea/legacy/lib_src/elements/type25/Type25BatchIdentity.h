// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Type25Batch.h"
#include "../../solvers/NodalTrialIdentity.h"

namespace tl::fea::type25::batch_detail {
BatchDiagnostics InitialDiagnostics(const BatchConfig&,std::uint64_t source_instance_id,double minimum_dt);
bool SameDiagnostics(const BatchDiagnostics&,const BatchDiagnostics&) noexcept;
} // namespace tl::fea::type25::batch_detail
