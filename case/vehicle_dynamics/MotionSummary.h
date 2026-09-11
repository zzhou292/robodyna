#pragma once
#include "lib_src/assembly/NodalCoefficientLedger.h"
#include "lib_src/solvers/FENodalState.h"
#include <vector>

namespace crash::cases::vehicle_dynamics {
struct Fields {
    std::vector<double> position,velocity,orientation,spin;
    explicit Fields(std::size_t n=0):position(3*n),velocity(3*n),orientation(4*n),spin(3*n) {}
    tl::fea::NodalSnapshotBuffer buffer() noexcept;
};
struct MotionSummary {
    std::size_t nodes=0;
    double maximum_position_error=0,maximum_velocity_error=0;
    double maximum_orientation_error=0,maximum_spin=0;
};
// Independent uniform translation observation from original coordinates.
// All physical slots, including dependent and absent-rotation nodes, are read.
// This computes errors; it neither prescribes motion nor changes coordinates.
MotionSummary ObserveUniformMotion(const tl::fea::NodalCoefficientLedger&,
                                  const Fields&,double speed,double time);
} // namespace crash::cases::vehicle_dynamics
