#pragma once
#include "../constraints/NodalRigidAccelerationSink.h"
#include <cstddef>

namespace tl::fea::nodal_detail {
// Appended after the existing 11*n scratch values, never in either state slab.
// n/g are the already admitted active owner counts; no capacity-sized arrays.
struct ForceStageCaptureLayout {
  std::size_t nodes,groups;
  std::size_t values() const noexcept {return 6*(nodes+groups);}
  std::size_t scratch_offset() const noexcept {return 11*nodes;}
  rigid::AccelerationSink Sink(double* scratch) const noexcept {
    auto* start=scratch+scratch_offset();
    return {start,start+3*nodes,start+6*nodes,start+6*nodes+3*groups};
  }
};
} // namespace tl::fea::nodal_detail
