// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "QbatReferenceCoefficients.h"
#include "lib_src/elements/qeph/QephStartup.h"

namespace tl::fea::qbat {
// Allocation-free startup. All failures preserve output, including when input
// aliases a previous output.input(). No batch, source or material admission.
// QBAT uses its own projection; a nondefault QEPH metric is unsupported.
TL_QBAT_HD inline Status InitializeReference(const ReferenceInput& input, Reference& output) {
  if (!detail::Supported(input.options) || !detail::Positive(input.initial_a11_pa) ||
      input.quadrilateral.placement != ShellReferencePlacement::Centered ||
      input.quadrilateral.projection_working_length_m != 1)
    return Status::kInvalidInput;
  Reference next;
  next.input_=input;
  // Exact common CNEVECI/CDERII and centered CINMAS FAC=12 selected by IHBE11.
  auto status=qeph::InitializeReference(input.quadrilateral,next.quad_);
  if (status!=Status::kSuccess) return status;
  status=detail::ReferenceCoefficients(input,next.quad_,next.coefficients_);
  if (status!=Status::kSuccess) return status;
  next.prepared_=true;
  output=next;
  return Status::kSuccess;
}
} // namespace tl::fea::qbat
