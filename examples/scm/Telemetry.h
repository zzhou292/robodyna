#pragma once

#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChLinkMotorRotationAngle.h"
#include "chrono_vehicle/terrain/SCMTerrain.h"
#include "output/ArtifactIO.h"

namespace robodyna::examples::scm {
// Observation only. It never changes bodies, loads, soil nodes or the clock.
class Telemetry {
  public:
    Telemetry(const chrono::ChVector3d& initial_position, double grid_spacing);
    void Observe(double time, const std::shared_ptr<chrono::ChBody>& wheel,
                 const chrono::ChLinkMotorRotationAngle& motor, const chrono::vehicle::SCMTerrain& terrain);
    crash::output::Document Finish(const chrono::vehicle::SCMTerrain& terrain) const;

  private:
    chrono::ChVector3d initial_, final_;
    double spacing_, time_ = 0, max_force_ = 0, max_depth_ = 0, max_plastic_ = 0, motor_angle_ = 0;
    std::uint64_t steps_ = 0, contact_steps_ = 0, soil_nodes_ = 0, max_ray_hits_ = 0;
};
}  // namespace robodyna::examples::scm
