#include "Telemetry.h"
#include <algorithm>
#include <array>
#include <cmath>

namespace robodyna::examples::mbd {
namespace {
namespace io = crash::output;

void CheckBody(const chrono::ChBody& body) {
    io::Require(std::isfinite(body.GetPos().Length2()) && std::isfinite(body.GetPosDt().Length2()) &&
                std::isfinite(body.GetAngVelParent().Length2()) && std::isfinite(body.GetRot().Length2()) &&
                body.GetRot().Length2() > 0, "Demo body state is nonfinite or has an invalid orientation");
}
void Position(io::Document& document, const char* name, const chrono::ChVector3d& value) {
    const std::array<double, 3> coordinates{value.x(), value.y(), value.z()};
    io::FiniteArray(document, name, coordinates.data(), coordinates.size());
}
io::Document Document(const char* model) {
    io::Document document;
    document.SetObject();
    io::String(document, "schema", "robodyna.original_mbd_observations.v1");
    io::String(document, "model", model);
    io::String(document, "dynamics_backend", "retained_chrono_cpu");
    io::Boolean(document, "cuda_dynamics", false);
    io::String(document, "scope", "observed original demo dynamics; not a new physical model or complete energy ledger");
    return document;
}
}  // namespace

SpringTelemetry::SpringTelemetry(chrono::ChBody& first, chrono::ChBody& second,
                                 double rest_length, double stiffness, double damping)
    : initial_first_(first.GetPos()), initial_second_(second.GetPos()),
      final_first_(first.GetPos()), final_second_(second.GetPos()), mass_(first.GetMass()),
      rest_(rest_length), stiffness_(stiffness), damping_(damping), initial_extension_(-first.GetPos().y() - rest_length) {
    io::Require(mass_ > 0 && second.GetMass() == mass_ && stiffness_ > 0 && damping_ >= 0 && rest_ > 0 &&
                first.GetPosDt().Length2() == 0 && second.GetPosDt().Length2() == 0 &&
                first.GetPos().y() == second.GetPos().y() && stiffness_ / mass_ > std::pow(damping_ / (2 * mass_), 2),
                "Original spring reference requires equal resting masses and the underdamped profile");
}

void SpringTelemetry::Observe(double time, const chrono::ChBody& first, const chrono::ChBody& second,
                              const chrono::ChLinkTSDA& spring_first, const chrono::ChLinkTSDA& spring_second) {
    io::Require(std::isfinite(time) && time >= final_time_, "Spring observation time is invalid");
    CheckBody(first);
    CheckBody(second);
    first_force_ = spring_first.GetForce();
    second_force_ = spring_second.GetForce();
    first_length_ = spring_first.GetLength();
    second_length_ = spring_second.GetLength();
    io::Require(std::isfinite(first_force_) && std::isfinite(second_force_) && std::isfinite(first_length_) &&
                std::isfinite(second_length_) && std::isfinite(spring_first.GetVelocity()) &&
                std::isfinite(spring_second.GetVelocity()), "Spring observation fields are nonfinite");
    const auto displacement_first = first.GetPos() - initial_first_;
    const auto displacement_second = second.GetPos() - initial_second_;
    peak_motion_ = std::max({peak_motion_, displacement_first.Length(), displacement_second.Length()});
    peak_speed_ = std::max({peak_speed_, first.GetPosDt().Length(), second.GetPosDt().Length()});
    position_agreement_ = std::max(position_agreement_, (displacement_first - displacement_second).Length());
    velocity_agreement_ = std::max(velocity_agreement_, (first.GetPosDt() - second.GetPosDt()).Length());
    length_agreement_ = std::max(length_agreement_, std::abs(first_length_ - second_length_));
    force_agreement_ = std::max(force_agreement_, std::abs(first_force_ - second_force_));
    // Diagnostic solution of the source demo's one-dimensional damped oscillator.
    // This value never feeds the mechanical system or visual body transforms.
    const double decay = damping_ / (2 * mass_);
    const double frequency_squared = stiffness_ / mass_;
    const double frequency = std::sqrt(frequency_squared - decay * decay);
    const double amplitude = initial_extension_ * std::exp(-decay * time);
    const double position = -rest_ - amplitude * (std::cos(frequency * time) + decay / frequency * std::sin(frequency * time));
    const double velocity = amplitude * frequency_squared / frequency * std::sin(frequency * time);
    analytic_position_error_ = std::max(analytic_position_error_, std::abs(first.GetPos().y() - position));
    analytic_velocity_error_ = std::max(analytic_velocity_error_, std::abs(first.GetPosDt().y() - velocity));
    final_first_ = first.GetPos();
    final_second_ = second.GetPos();
    final_time_ = time;
    ++observations_;
}

io::Document SpringTelemetry::Summary() const {
    auto document = Document("original_demo_MBS_spring");
    io::Integer(document, "observed_states", observations_);
    io::Number(document, "final_time_s", final_time_);
    io::Number(document, "mass_each_kg", mass_);
    io::Number(document, "stiffness_n_per_m", stiffness_);
    io::Number(document, "damping_ns_per_m", damping_);
    io::Number(document, "rest_length_m", rest_);
    Position(document, "initial_first_position_m", initial_first_);
    Position(document, "initial_second_position_m", initial_second_);
    Position(document, "final_first_position_m", final_first_);
    Position(document, "final_second_position_m", final_second_);
    io::Number(document, "maximum_displacement_m", peak_motion_);
    io::Number(document, "maximum_speed_m_per_s", peak_speed_);
    io::Number(document, "native_callback_position_difference_m", position_agreement_);
    io::Number(document, "native_callback_velocity_difference_m_per_s", velocity_agreement_);
    io::Number(document, "native_callback_length_difference_m", length_agreement_);
    io::Number(document, "native_callback_force_difference_n", force_agreement_);
    io::Number(document, "maximum_analytic_position_error_m", analytic_position_error_);
    io::Number(document, "maximum_analytic_velocity_error_m_per_s", analytic_velocity_error_);
    io::Number(document, "final_native_force_n", first_force_);
    io::Number(document, "final_callback_force_n", second_force_);
    io::Number(document, "final_native_length_m", first_length_);
    io::Number(document, "final_callback_length_m", second_length_);
    io::String(document, "analytic_scope", "continuous damped oscillator reference; numerical errors reported, not prescribed motion");
    return document;
}

CollisionTelemetry::CollisionTelemetry(const chrono::ChSystem& system, std::uint32_t seed) : seed_(seed) {
    io::Require(system.GetBodies().size() == 93, "Original collision demo must retain all 87 objects, five walls and mixer");
    for (const auto& body : system.GetBodies()) {
        CheckBody(*body);
        initial_positions_.push_back(body->GetPos());
        identities_.push_back(body.get());
        if (body->IsFixed()) ++fixed_bodies_;
        else { ++dynamic_bodies_; total_mass_ += body->GetMass(); }
    }
    io::Require(fixed_bodies_ == 5 && dynamic_bodies_ == 88, "Original collision body roles differ");
}

void CollisionTelemetry::Observe(const chrono::ChSystem& system) {
    const auto& bodies = system.GetBodies();
    io::Require(bodies.size() == identities_.size() && std::isfinite(system.GetChTime()) &&
                system.GetChTime() >= final_time_, "Collision body inventory or observation time changed");
    for (std::size_t i = 0; i < bodies.size(); ++i) {
        io::Require(bodies[i].get() == identities_[i], "Collision body identity/order changed");
        CheckBody(*bodies[i]);
        peak_motion_ = std::max(peak_motion_, (bodies[i]->GetPos() - initial_positions_[i]).Length());
        peak_speed_ = std::max(peak_speed_, bodies[i]->GetPosDt().Length());
    }
    const auto contacts = system.GetNumContacts();
    peak_contacts_ = std::max(peak_contacts_, contacts);
    if (contacts) ++contact_states_;
    final_time_ = system.GetChTime();
    ++observations_;
}

io::Document CollisionTelemetry::Summary() const {
    auto document = Document("original_demo_MBS_collisionNSC");
    io::Integer(document, "random_seed", seed_);
    io::Integer(document, "falling_objects", 87);
    io::Integer(document, "fixed_bodies", fixed_bodies_);
    io::Integer(document, "dynamic_bodies_including_mixer", dynamic_bodies_);
    io::Number(document, "dynamic_mass_kg", total_mass_);
    io::Integer(document, "observed_states", observations_);
    io::Number(document, "final_time_s", final_time_);
    io::Integer(document, "states_with_contacts", contact_states_);
    io::Integer(document, "maximum_reported_contacts", peak_contacts_);
    io::Number(document, "maximum_displacement_m", peak_motion_);
    io::Number(document, "maximum_speed_m_per_s", peak_speed_);
    io::String(document, "contact_backend", "retained_CPU_Bullet_NSC_PSOR50");
    io::String(document, "seed_scope", "reproducible ChRandom initialization for this build; not cross-platform bitwise dynamics certification");
    return document;
}
}  // namespace robodyna::examples::mbd
