#include "Telemetry.h"

#include <algorithm>
#include <cmath>

namespace robodyna::examples::scm {
namespace {
bool Finite(const chrono::ChVector3d& value) {
    return std::isfinite(value.x()) && std::isfinite(value.y()) && std::isfinite(value.z());
}
}

Telemetry::Telemetry(const chrono::ChVector3d& initial_position, double grid_spacing)
    : initial_(initial_position), final_(initial_position), spacing_(grid_spacing) {}

void Telemetry::Observe(double time, const std::shared_ptr<chrono::ChBody>& wheel,
                        const chrono::ChLinkMotorRotationAngle& motor, const chrono::vehicle::SCMTerrain& terrain) {
    using crash::output::Require;
    Require(std::isfinite(time) && time > time_, "SCM telemetry clock did not advance");
    Require(wheel && Finite(wheel->GetPos()) && Finite(wheel->GetPosDt()) && Finite(wheel->GetAngVelParent()),
            "SCM wheel state is not finite");
    const auto& rotation = wheel->GetRot();
    Require(std::isfinite(rotation.e0()) && std::isfinite(rotation.e1()) &&
            std::isfinite(rotation.e2()) && std::isfinite(rotation.e3()), "SCM wheel rotation is not finite");
    Require(std::isfinite(motor.GetMotorAngle()) && std::isfinite(motor.GetMotorTorque()),
            "SCM motor state is not finite");
    chrono::ChVector3d force(0), torque(0);
    const bool contact = terrain.GetContactForceBody(wheel, force, torque);
    Require(Finite(force) && Finite(torque), "SCM contact wrench is not finite");
    max_force_ = std::max(max_force_, force.Length());
    contact_steps_ += contact;
    Require(terrain.GetNumRayHits() >= 0, "SCM reported a negative ray-hit count");
    max_ray_hits_ = std::max(max_ray_hits_, static_cast<std::uint64_t>(terrain.GetNumRayHits()));
    const auto nodes = terrain.GetModifiedNodes(true);
    soil_nodes_ = nodes.size();
    for (const auto& node : nodes) {
        Require(std::isfinite(node.second), "SCM soil height is not finite");
        max_depth_ = std::max(max_depth_, -node.second);  // Original flat plane is local z = 0.
        const auto point = terrain.GetReferenceFrame().TransformPointLocalToParent(
            {node.first.x() * spacing_, node.first.y() * spacing_, 0});
        const auto state = terrain.GetNodeInfo(point);
        Require(std::isfinite(state.sinkage) && std::isfinite(state.sinkage_plastic) &&
                std::isfinite(state.sinkage_elastic) && std::isfinite(state.sigma) &&
                std::isfinite(state.sigma_yield) && std::isfinite(state.kshear) && std::isfinite(state.tau),
                "SCM soil state is not finite");
        max_plastic_ = std::max(max_plastic_, state.sinkage_plastic);
    }
    Require(terrain.GetNumRaycastGpuSteps() == 0 && terrain.GetNumContactForceGpuSteps() == 0,
            "SCM retention run changed from the declared CPU/Bullet profile");
    final_ = wheel->GetPos();
    motor_angle_ = motor.GetMotorAngle();
    time_ = time;
    ++steps_;
}

crash::output::Document Telemetry::Finish(const chrono::vehicle::SCMTerrain& terrain) const {
    namespace output = crash::output;
    output::Require(steps_ > 0, "SCM demo accepted no physical steps");
    const double travel = std::hypot(final_.x() - initial_.x(), final_.z() - initial_.z());
    const bool contact = contact_steps_ > 0 && max_force_ > 0 && soil_nodes_ > 0;
    const bool rut = max_depth_ > 0 && max_plastic_ > 0;
    // Short smoke runs may stop before the original wheel reaches the soil.
    // A requested multi-second demonstration must actually contact and deform it.
    if (time_ >= 1)
        output::Require(contact && rut && travel > .01, "SCM demo did not show wheel travel and permanent soil deformation");
    output::Document result(rapidjson::kObjectType);
    output::String(result, "model", "retained_chrono_scm_lugged_rigid_tire");
    output::String(result, "physics_backend", "CPU SCM with Bullet ray casting");
    output::Integer(result, "observed_steps", steps_);
    output::Number(result, "time_s", time_);
    output::FiniteArray(result, "initial_wheel_position_m", initial_.data(), 3);
    output::FiniteArray(result, "final_wheel_position_m", final_.data(), 3);
    output::Number(result, "horizontal_wheel_travel_m", travel);
    output::Number(result, "motor_angle_rad", motor_angle_);
    output::Number(result, "peak_contact_force_n", max_force_);
    output::Number(result, "maximum_soil_depression_m", max_depth_);
    output::Number(result, "maximum_plastic_sinkage_m", max_plastic_);
    output::Integer(result, "contact_steps", contact_steps_);
    output::Integer(result, "stored_soil_nodes", soil_nodes_);
    output::Integer(result, "peak_ray_hits", max_ray_hits_);
    output::Integer(result, "gpu_ray_steps", terrain.GetNumRaycastGpuSteps());
    output::Integer(result, "gpu_force_steps", terrain.GetNumContactForceGpuSteps());
    output::Boolean(result, "contact_observed", contact);
    output::Boolean(result, "permanent_rut_observed", rut);
    return result;
}
}  // namespace robodyna::examples::scm
