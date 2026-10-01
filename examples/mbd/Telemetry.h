#pragma once

#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChLinkTSDA.h"
#include "chrono/physics/ChSystem.h"
#include "output/ArtifactIO.h"
#include <cstdint>
#include <vector>

namespace robodyna::examples::mbd {

// Read-only diagnostics of the original demos. No applied force, timestep or
// integrator is owned or modified by these observers.
class SpringTelemetry {
  public:
    SpringTelemetry(chrono::ChBody& first, chrono::ChBody& second,
                    double rest_length, double stiffness, double damping);
    void Observe(double time, const chrono::ChBody& first, const chrono::ChBody& second,
                 const chrono::ChLinkTSDA& spring_first, const chrono::ChLinkTSDA& spring_second);
    crash::output::Document Summary() const;
  private:
    chrono::ChVector3d initial_first_, initial_second_, final_first_, final_second_;
    double mass_, rest_, stiffness_, damping_, initial_extension_;
    std::uint64_t observations_ = 0;
    double final_time_ = 0, peak_motion_ = 0, peak_speed_ = 0;
    double position_agreement_ = 0, velocity_agreement_ = 0, length_agreement_ = 0, force_agreement_ = 0;
    double analytic_position_error_ = 0, analytic_velocity_error_ = 0;
    double first_force_ = 0, second_force_ = 0, first_length_ = 0, second_length_ = 0;
};

class CollisionTelemetry {
  public:
    CollisionTelemetry(const chrono::ChSystem& system, std::uint32_t seed);
    void Observe(const chrono::ChSystem& system);
    crash::output::Document Summary() const;
  private:
    std::vector<chrono::ChVector3d> initial_positions_;
    std::vector<const chrono::ChBody*> identities_;
    std::uint32_t seed_;
    std::size_t dynamic_bodies_ = 0, fixed_bodies_ = 0;
    std::uint64_t observations_ = 0, contact_states_ = 0;
    unsigned peak_contacts_ = 0;
    double final_time_ = 0, peak_motion_ = 0, peak_speed_ = 0, total_mass_ = 0;
};

}  // namespace robodyna::examples::mbd
