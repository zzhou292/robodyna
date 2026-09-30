// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "QbatBatchTypes.h"
#include "../../solvers/NodalTrialIdentity.h"

namespace tl::fea::qbat::batch_detail {
bool SameDiagnostics(const BatchDiagnostics&,const BatchDiagnostics&) noexcept;
BatchDiagnostics CandidateIdentity(const BatchConfig&,const NodalPreparedView&) noexcept;
} // namespace tl::fea::qbat::batch_detail
