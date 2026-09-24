// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Batch.h"
#include "../RepresentedIntervalCrossingGpu.h"

namespace tlfea::contact::represented_interval_crossing {
struct DeviceBatchReport {
  BatchReport native;
  RepresentedIntervalDeviceReport device;
};
// Internal compound operation over the same native batch helpers. One facade
// and native busy lease spans the complete cohort. All paths authenticate once;
// the first eligible slice uploads the scene once, and only pair jobs/results
// move for later slices. The native lexical scene expires before return.
// Failed later slices preserve the preceding native publication, while the
// outer caller scratch is never advertised complete. The input and private
// scratch borrowing/strict canonical pair rules are identical to BatchAccess.
struct DeviceBatchAccess {
  static DeviceBatchReport Certify(RepresentedIntervalCrossingGpu&,
      const RepresentedTrianglePath*, std::size_t, const RepresentedTrianglePair*,
      std::size_t, std::size_t batch_pair_capacity, RepresentedIntervalResult*,
      std::size_t scratch_capacity, cudaStream_t) noexcept;
};
}  // namespace tlfea::contact::represented_interval_crossing
