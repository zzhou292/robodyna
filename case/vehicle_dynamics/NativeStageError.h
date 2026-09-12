#pragma once

#include "lib_src/elements/beam18/resident/Batch.h"
#include "lib_src/elements/solids/resident/Batch.h"
#include <stdexcept>
#include <variant>

namespace crash::cases::vehicle_dynamics {

// Copies only values from the rejected report. Native message pointers and
// source/model/owner storage are not retained. Indices are not source IDs.
struct SolidFailure {
    tl::fea::solids::BatchStatus status;
    tl::fea::solids::Family family;
    std::size_t parent_index;
    std::size_t node_index;
    int element_status;
    tl::fea::NodalStatus nodal_status;
};

struct StructuralBeamFailure {
    tl::fea::beam18::BatchStatus status;
    std::size_t parent_index;
    std::size_t node_index;
    int element_status;
    tl::fea::NodalStatus nodal_status;
};

using NativeFailure = std::variant<SolidFailure, StructuralBeamFailure>;

class NativeStageError : public std::runtime_error {
  public:
    NativeStageError(const tl::fea::solids::BatchReport&, const char* stage);
    NativeStageError(const tl::fea::beam18::BatchReport&, const char* stage);

    const NativeFailure& failure() const noexcept { return failure_; }

  private:
    NativeFailure failure_;
};

} // namespace crash::cases::vehicle_dynamics
