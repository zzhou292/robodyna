#include "ElasticCouponCase.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"
#include "chrono/core/ChQuaternion.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <sstream>

namespace {
using namespace crash::case_data;
namespace ref = crash::reference;
namespace shell = tl::fea::reissner;
using Vec = chrono::ChVector3d;
using Frame = ElasticCouponFrame;

class ElasticCoupon : public ::testing::Test {
  protected:
    void SetUp() override {
        int count = 0;
        ASSERT_EQ(cudaGetDeviceCount(&count), cudaSuccess);
        ASSERT_GT(count, 0);  // This target is actual CUDA evidence, never a CPU substitute or silent skip.
    }
};

std::string Precise(double value) {
    std::ostringstream text; text.precision(17); text << value; return text.str();
}
Vec Read(const std::array<double, 3 * ref::kCouponNodes>& data, std::size_t n) {
    return {data[3*n], data[3*n+1], data[3*n+2]};
}
double Tip(const Frame& frame) { return .5 * (frame.position[14] + frame.position[17]); }
double Energy(const Frame& frame) {
    return frame.metrics.diagnostics.elastic_energy + CouponKineticEnergy(frame.metrics.diagnostics);
}
void ExpectVector(const Vec& actual, const Vec& expected, double absolute, double relative = 1e-9) {
    EXPECT_LE((actual - expected).Length(), absolute + relative * expected.Length());
}
void ExpectClamps(const Frame& frame, const Frame& initial) {
    for (const auto n : {1, 2}) {
        for (unsigned axis = 0; axis < 3; ++axis) {
            EXPECT_EQ(frame.position[3*n+axis], initial.position[3*n+axis]);
            EXPECT_EQ(frame.velocity[3*n+axis], 0);
            EXPECT_EQ(frame.omega[3*n+axis], 0);
        }
        for (unsigned c = 0; c < 4; ++c) EXPECT_EQ(frame.rotation[4*n+c], initial.rotation[4*n+c]);
    }
}
std::array<double, 23> Scalars(const shell::ShellBatchDiagnostics& d) {
    return {{d.elastic_energy, d.bending_energy, d.kinetic_translation, d.kinetic_physical_rotation,
             d.kinetic_artificial_drilling, d.maximum_displacement, d.maximum_director_departure,
             d.maximum_pair_angle, d.maximum_membrane_strain, d.maximum_thickness_curvature,
             d.minimum_signed_area_ratio, d.minimum_area_norm_ratio, d.maximum_area_norm_ratio,
             d.minimum_display_triangle_area_ratio, d.base_elastic_energy, d.base_kinetic_energy,
             d.elastic_energy_increment, d.kinetic_energy_increment, d.kinetic_midpoint_work,
             d.kinetic_work_residual, d.force_coordinate_work, d.conservative_force_coordinate_defect,
             d.mass_weighted_increment_squared}};
}
void ExpectAcceptedEqual(const Frame& actual, const Frame& expected, bool same_owner) {
    EXPECT_EQ(actual.position, expected.position); EXPECT_EQ(actual.velocity, expected.velocity);
    EXPECT_EQ(actual.rotation, expected.rotation); EXPECT_EQ(actual.omega, expected.omega);
    EXPECT_EQ(actual.reaction_force, expected.reaction_force); EXPECT_EQ(actual.reaction_couple, expected.reaction_couple);
    const auto& a = actual.stamp; const auto& b = expected.stamp;
    EXPECT_EQ(a.epoch, b.epoch); EXPECT_EQ(a.time, b.time); EXPECT_EQ(a.fixed_dt, b.fixed_dt);
    EXPECT_EQ(a.node_count, b.node_count); EXPECT_EQ(a.has_rotations, b.has_rotations);
    EXPECT_EQ(a.reactions_valid, b.reactions_valid); EXPECT_EQ(a.reaction_base_epoch, b.reaction_base_epoch);
    EXPECT_EQ(a.reaction_time, b.reaction_time);
    EXPECT_EQ(Scalars(actual.metrics.diagnostics), Scalars(expected.metrics.diagnostics));
    EXPECT_EQ(actual.metrics.initial_energy, expected.metrics.initial_energy);
    EXPECT_EQ(actual.metrics.maximum_relative_energy_error, expected.metrics.maximum_relative_energy_error);
    EXPECT_EQ(actual.metrics.last_operator_norm, expected.metrics.last_operator_norm);
    EXPECT_EQ(actual.metrics.last_operator_epoch, expected.metrics.last_operator_epoch);
    EXPECT_EQ(actual.metrics.required_steps, expected.metrics.required_steps);
    EXPECT_EQ(actual.metrics.full_state_audit_reads, expected.metrics.full_state_audit_reads);
    const auto& da = actual.metrics.diagnostics; const auto& db = expected.metrics.diagnostics;
    EXPECT_EQ(da.base_epoch, db.base_epoch); EXPECT_EQ(da.configuration_id, db.configuration_id);
    EXPECT_EQ(da.phase, db.phase); EXPECT_EQ(da.valid, db.valid);
    if (same_owner) { EXPECT_EQ(a.owner_id, b.owner_id); EXPECT_EQ(da.owner_id, db.owner_id); EXPECT_EQ(da.attempt, db.attempt); }
    for (std::size_t e = 0; e < ref::kCouponElements; ++e) {
        EXPECT_EQ(actual.element[e].energy, expected.element[e].energy);
        EXPECT_EQ(actual.element[e].bending_energy, expected.element[e].bending_energy);
        for (std::size_t n = 0; n < 4; ++n) {
            for (std::size_t c = 0; c < 3; ++c) {
                EXPECT_EQ(shell::detail::Component(actual.element[e].force[n], c),
                          shell::detail::Component(expected.element[e].force[n], c));
                EXPECT_EQ(shell::detail::Component(actual.element[e].couple[n], c),
                          shell::detail::Component(expected.element[e].couple[n], c));
            }
            for (std::size_t c = 0; c < 12; ++c) {
                EXPECT_EQ(actual.element[e].strain[n][c], expected.element[e].strain[n][c]);
                EXPECT_EQ(actual.element[e].resultant[n][c], expected.element[e].resultant[n][c]);
            }
        }
    }
}
void ExpectAllocations(const ElasticCouponCase& run, tl::fea::NodalAllocationInfo state,
                       tl::fea::NodalAllocationInfo element) {
    EXPECT_EQ(run.state_allocations().device_bytes, state.device_bytes);
    EXPECT_EQ(run.state_allocations().device_allocations, state.device_allocations);
    EXPECT_EQ(run.element_allocations().device_bytes, element.device_bytes);
    EXPECT_EQ(run.element_allocations().device_allocations, element.device_allocations);
}

// Independent nodal momentum/energy ledger. The known flat coupon has a common
// physical director at each shared node; use its actual immutable frame offset.
struct Ledger { Vec linear, angular; double translation = 0, physical = 0, drilling = 0; };
Ledger Measure(const Frame& frame, const ref::ElasticCouponData& model) {
    Ledger result;
    for (std::size_t n = 0; n < ref::kCouponNodes; ++n) {
        const auto& mass = model.nodal_mass[n];
        const auto x = Read(frame.position, n), v = Read(frame.velocity, n), omega = Read(frame.omega, n);
        const Vec momentum = mass.mass * v;
        result.linear += momentum;
        result.angular += chrono::Vcross(x, momentum) + mass.physical_tangential_inertia * omega;
        shell::Quaternion offset;
        bool found = false;
        for (std::size_t e = 0; e < ref::kCouponElements && !found; ++e)
            for (std::size_t local = 0; local < 4; ++local)
                if (model.connectivity[e][local] == n) { offset = model.reference[e].node_frame_offset[local]; found = true; break; }
        const chrono::ChQuaterniond q(frame.rotation[4*n], frame.rotation[4*n+1], frame.rotation[4*n+2], frame.rotation[4*n+3]);
        const auto physical = q * chrono::ChQuaterniond(offset.w, offset.x, offset.y, offset.z);
        const Vec director = physical.Rotate(Vec(0, 0, 1));
        const double spin = omega.Dot(director);
        result.translation += .5 * mass.mass * v.Length2();
        result.physical += .5 * mass.physical_tangential_inertia * chrono::Vcross(omega, director).Length2();
        result.drilling += .5 * mass.artificial_drilling_inertia * spin * spin;
    }
    return result;
}

TEST_F(ElasticCoupon, InvalidLifecycleAndConfigurationPublishNothing) {
    ElasticCouponCase run; Frame sentinel; sentinel.position[0] = 123;
    EXPECT_EQ(run.Step().status, CouponStatus::NotInitialized);
    EXPECT_EQ(run.Capture(sentinel).status, CouponStatus::NotInitialized);
    EXPECT_EQ(sentinel.position[0], 123);
    for (const ElasticCouponConfig config : {ElasticCouponConfig{0,20}, {3,20}, {1,0}, {1,65}}) {
        EXPECT_EQ(run.Initialize(config).status, CouponStatus::InvalidInput);
        EXPECT_EQ(run.metrics(), nullptr); EXPECT_EQ(run.modal(), nullptr); EXPECT_EQ(run.output(), nullptr);
        EXPECT_EQ(run.state_allocations().device_bytes, 0u); EXPECT_EQ(run.element_allocations().device_bytes, 0u);
    }
}

TEST_F(ElasticCoupon, HundredForceDrivenStepsPreserveAllocationAndClamps) {
    ElasticCouponCase run;
    auto report = run.Initialize(); ASSERT_EQ(report.status, CouponStatus::Ok) << report.diagnostic;
    Frame initial, after;
    report = run.Capture(initial); ASSERT_EQ(report.status, CouponStatus::Ok) << report.diagnostic;
    const auto state = run.state_allocations(), elements = run.element_allocations();
    EXPECT_GT(state.device_bytes, 0u); EXPECT_GT(elements.device_bytes, 0u);
    EXPECT_LE(state.device_bytes, tl::fea::MaxTranslationDeviceBytes);
    EXPECT_LE(elements.device_bytes, shell::MaxReissnerShellBatchDeviceBytes);
    ASSERT_GT(run.metrics()->required_steps, 100u);
    EXPECT_NEAR(Tip(initial), .002, 1e-15);
    EXPECT_EQ(CouponKineticEnergy(initial.metrics.diagnostics), 0);
    const auto start = std::chrono::steady_clock::now();
    for (unsigned step = 1; step <= 100; ++step) {
        report = run.Step(); ASSERT_EQ(report.status, CouponStatus::Ok) << report.diagnostic << " step " << step;
        ASSERT_EQ(run.metrics()->stamp.epoch, step);
        EXPECT_EQ(run.metrics()->diagnostics.base_epoch, step - 1);
        EXPECT_EQ(run.metrics()->diagnostics.phase, shell::ShellBatchPhase::kPreparedCandidate);
        EXPECT_LE(run.metrics()->maximum_relative_energy_error, ElasticCouponLimits::energy_fraction);
        EXPECT_EQ(run.output()->surface().frame()->epoch, 0u);
    }
    const double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
    report = run.Capture(after); ASSERT_EQ(report.status, CouponStatus::Ok) << report.diagnostic;
    ExpectClamps(after, initial); ExpectAllocations(run, state, elements);
    EXPECT_LT(Tip(after), Tip(initial) - 1e-9);
    EXPECT_GT(CouponKineticEnergy(after.metrics.diagnostics), 0);
    EXPECT_NE(after.rotation, initial.rotation);
    EXPECT_NE(after.element[0].strain[0][7], initial.element[0].strain[0][7]);
    EXPECT_NEAR(after.stamp.time, 100 * after.stamp.fixed_dt, 1e-14);
    const auto& surface = run.output()->surface();
    ASSERT_EQ(surface.frame()->epoch, 100u); ASSERT_EQ(surface.binding()->triangles.size(), 4u);
    ASSERT_EQ(surface.mesh()->GetCoordsVertices().size(), ref::kCouponNodes);
    for (std::size_t n = 0; n < ref::kCouponNodes; ++n)
        ExpectVector(surface.mesh()->GetCoordsVertices()[n], Read(after.position, n), 0, 0);
    EXPECT_EQ(run.Initialize().status, CouponStatus::AlreadyInitialized);
    RecordProperty("hundred_steps_wall_seconds", Precise(elapsed));
    RecordProperty("time_step", Precise(after.stamp.fixed_dt));
    RecordProperty("required_steps", std::to_string(after.metrics.required_steps));
    RecordProperty("tip_initial", Precise(Tip(initial))); RecordProperty("tip_after_100", Precise(Tip(after)));
    RecordProperty("maximum_energy_error", Precise(after.metrics.maximum_relative_energy_error));
}

TEST_F(ElasticCoupon, LateRejectionCapturePreservesAcceptedFrameAndCleanRetry) {
    ElasticCouponCase run, clean;
    auto report = run.Initialize(); ASSERT_EQ(report.status, CouponStatus::Ok) << report.diagnostic;
    report = clean.Initialize(); ASSERT_EQ(report.status, CouponStatus::Ok) << report.diagnostic;
    for (unsigned step = 0; step < 4; ++step) {
        report = run.Step(); ASSERT_EQ(report.status, CouponStatus::Ok) << report.diagnostic;
        report = clean.Step(); ASSERT_EQ(report.status, CouponStatus::Ok) << report.diagnostic;
    }
    Frame before, recovered, retried, baseline;
    report = run.Capture(before); ASSERT_EQ(report.status, CouponStatus::Ok) << report.diagnostic;
    const auto shown = run.output()->surface().mesh()->GetCoordsVertices();
    const auto state = run.state_allocations(), elements = run.element_allocations();
    report = run.Step({1e-9}); EXPECT_EQ(report.status, CouponStatus::AdmissionFailure) << report.diagnostic;
    EXPECT_NE(report.diagnostic.find("displacement"), std::string::npos);
    EXPECT_EQ(run.metrics()->stamp.epoch, before.stamp.epoch);
    EXPECT_EQ(run.metrics()->diagnostics.attempt, before.metrics.diagnostics.attempt);
    EXPECT_EQ(run.output()->surface().mesh()->GetCoordsVertices(), shown);
    report = run.Capture(recovered); ASSERT_EQ(report.status, CouponStatus::Ok) << report.diagnostic;
    ExpectAcceptedEqual(recovered, before, true);
    EXPECT_EQ(recovered.element_association.phase, shell::ShellBatchPhase::kAcceptedBase);
    EXPECT_EQ(recovered.element_association.base_epoch, before.stamp.epoch);
    EXPECT_GT(recovered.element_association.attempt, before.element_association.attempt);
    EXPECT_EQ(recovered.metrics.diagnostics.phase, shell::ShellBatchPhase::kPreparedCandidate);
    EXPECT_EQ(recovered.metrics.diagnostics.base_epoch, before.stamp.epoch - 1);
    EXPECT_EQ(run.output()->surface().mesh()->GetCoordsVertices(), shown);
    report = run.Step(); ASSERT_EQ(report.status, CouponStatus::Ok) << report.diagnostic;
    report = clean.Step(); ASSERT_EQ(report.status, CouponStatus::Ok) << report.diagnostic;
    report = run.Capture(retried); ASSERT_EQ(report.status, CouponStatus::Ok) << report.diagnostic;
    report = clean.Capture(baseline); ASSERT_EQ(report.status, CouponStatus::Ok) << report.diagnostic;
    ExpectAcceptedEqual(retried, baseline, false);
    ExpectAllocations(run, state, elements);
}

TEST_F(ElasticCoupon, CoupledImpulseReactionsAndSeparatedKineticEnergy) {
    ref::ElasticCouponModel model;
    ElasticCouponCase run;
    auto report = run.Initialize(); ASSERT_EQ(report.status, CouponStatus::Ok) << report.diagnostic;
    Frame initial, before, after;
    report = run.Capture(initial); ASSERT_EQ(report.status, CouponStatus::Ok) << report.diagnostic;
    before = initial;
    Vec linear_impulse, angular_impulse;
    for (unsigned step = 1; step <= 12; ++step) {
        SCOPED_TRACE(step);
        report = run.Step(); ASSERT_EQ(report.status, CouponStatus::Ok) << report.diagnostic;
        report = run.Capture(after); ASSERT_EQ(report.status, CouponStatus::Ok) << report.diagnostic;
        const auto old = Measure(before, model.data()), next = Measure(after, model.data());
        Vec reaction_force, reaction_moment;
        double clamp_work = 0;
        for (std::size_t n = 0; n < ref::kCouponNodes; ++n) {
            const Vec force = Read(after.reaction_force, n), couple = Read(after.reaction_couple, n);
            reaction_force += force;
            reaction_moment += chrono::Vcross(Read(before.position, n), force) + couple;
            if (model.data().fixed[n]) {
                Vec base_force, base_couple;
                for (std::size_t e = 0; e < ref::kCouponElements; ++e)
                    for (std::size_t local = 0; local < 4; ++local)
                        if (model.data().connectivity[e][local] == n) {
                            const auto f = before.element[e].force[local], c = before.element[e].couple[local];
                            base_force += Vec(f.x, f.y, f.z); base_couple += Vec(c.x, c.y, c.z);
                        }
                ExpectVector(force, -base_force, 1e-11); ExpectVector(couple, -base_couple, 1e-12);
                clamp_work += force.Dot(Read(after.position, n) - Read(before.position, n)) +
                              after.stamp.fixed_dt * couple.Dot(Read(after.omega, n));
            } else { ExpectVector(force, Vec(0), 0, 0); ExpectVector(couple, Vec(0), 0, 0); }
        }
        const double h = after.stamp.fixed_dt;
        ExpectVector(next.linear - old.linear, h * reaction_force, 1e-12);
        ExpectVector(next.angular - old.angular, h * reaction_moment, 1e-13);
        linear_impulse += h * reaction_force; angular_impulse += h * reaction_moment;
        EXPECT_EQ(clamp_work, 0); ExpectClamps(after, initial);
        EXPECT_TRUE(after.stamp.reactions_valid);
        EXPECT_EQ(after.stamp.reaction_base_epoch, before.stamp.epoch);
        EXPECT_EQ(after.stamp.reaction_time, before.stamp.time);
        const auto& d = after.metrics.diagnostics;
        EXPECT_NEAR(next.translation, d.kinetic_translation, 1e-15);
        EXPECT_NEAR(next.physical, d.kinetic_physical_rotation, 1e-15);
        EXPECT_NEAR(next.drilling, d.kinetic_artificial_drilling, 1e-15);
        before = after;
    }
    const auto start = Measure(initial, model.data()), finish = Measure(after, model.data());
    ExpectVector(finish.linear - start.linear, linear_impulse, 2e-12);
    ExpectVector(finish.angular - start.angular, angular_impulse, 2e-13);
    EXPECT_GT(linear_impulse.Length(), 1e-10); EXPECT_GT(angular_impulse.Length(), 1e-11);
}

TEST_F(ElasticCoupon, DeclaredSpectralAuditAndOutputCadencesRemainSeparate) {
    ElasticCouponCase run;
    auto report = run.Initialize({1,64}); ASSERT_EQ(report.status, CouponStatus::Ok) << report.diagnostic;
    const std::uint64_t stride = (run.modal()->step_count + 63) / 64;
    ASSERT_GT(stride, 1u);
    const auto state = run.state_allocations(), elements = run.element_allocations();
    for (std::uint64_t step = 1; step <= stride + 1; ++step) {
        report = run.Step(); ASSERT_EQ(report.status, CouponStatus::Ok) << report.diagnostic << " step " << step;
        EXPECT_EQ(run.metrics()->full_state_audit_reads, step >= stride ? 1u : 0u);
        EXPECT_EQ(run.metrics()->last_operator_epoch, step >= stride ? stride : 0u);
        EXPECT_EQ(run.output()->surface().frame()->epoch, 0u);
    }
    Frame shown;
    report = run.Capture(shown); ASSERT_EQ(report.status, CouponStatus::Ok) << report.diagnostic;
    EXPECT_EQ(run.output()->surface().frame()->epoch, stride + 1);
    EXPECT_LE(shown.metrics.last_operator_norm, run.modal()->monitored_norm_limit);
    report = run.Capture(shown); ASSERT_EQ(report.status, CouponStatus::Ok) << report.diagnostic;
    EXPECT_EQ(run.metrics()->full_state_audit_reads, 1u);
    ExpectAllocations(run, state, elements);
}

TEST_F(ElasticCoupon, CandidateAdmissionRejectsMalformedIntervalLedgers) {
    ElasticCouponCase run;
    auto report = run.Initialize(); ASSERT_EQ(report.status, CouponStatus::Ok) << report.diagnostic;
    report = run.Step(); ASSERT_EQ(report.status, CouponStatus::Ok) << report.diagnostic;
    const auto valid = run.metrics()->diagnostics;
    std::string error;
    ASSERT_TRUE(CheckElasticCouponEnvelope(valid, *run.modal(), run.metrics()->initial_energy,
                                           ElasticCouponLimits::displacement, true, error)) << error;
    using Member = double shell::ShellBatchDiagnostics::*;
    const std::array<Member, 9> fields{{&shell::ShellBatchDiagnostics::base_elastic_energy,
        &shell::ShellBatchDiagnostics::base_kinetic_energy, &shell::ShellBatchDiagnostics::elastic_energy_increment,
        &shell::ShellBatchDiagnostics::kinetic_energy_increment, &shell::ShellBatchDiagnostics::kinetic_midpoint_work,
        &shell::ShellBatchDiagnostics::force_coordinate_work, &shell::ShellBatchDiagnostics::kinetic_work_residual,
        &shell::ShellBatchDiagnostics::conservative_force_coordinate_defect,
        &shell::ShellBatchDiagnostics::mass_weighted_increment_squared}};
    for (const auto field : fields) {
        auto corrupt = valid; corrupt.*field = std::numeric_limits<double>::quiet_NaN();
        EXPECT_FALSE(CheckElasticCouponEnvelope(corrupt, *run.modal(), run.metrics()->initial_energy,
                                                ElasticCouponLimits::displacement, true, error));
        EXPECT_FALSE(error.empty());
    }
    for (const auto field : {&shell::ShellBatchDiagnostics::base_elastic_energy,
                              &shell::ShellBatchDiagnostics::base_kinetic_energy}) {
        auto corrupt = valid; corrupt.*field = -1;
        EXPECT_FALSE(CheckElasticCouponEnvelope(corrupt, *run.modal(), run.metrics()->initial_energy,
                                                ElasticCouponLimits::displacement, true, error));
    }
    EXPECT_EQ(Scalars(run.metrics()->diagnostics), Scalars(valid));
}

struct Trajectory {
    std::array<double, 41> time{}, tip{}, energy{};
    double initial_energy = 0, maximum_energy_error = 0, crossing_time = 0, minimum_time = 0;
};
bool RunHalfPeriod(unsigned refinement, Trajectory& result) {
    ElasticCouponCase run;
    auto report = run.Initialize({refinement,20});
    if (report.status != CouponStatus::Ok) { ADD_FAILURE() << report.diagnostic; return false; }
    const auto base_steps = run.modal()->step_count;
    const auto state = run.state_allocations(), element = run.element_allocations();
    result.initial_energy = run.metrics()->initial_energy;
    for (std::size_t sample = 0; sample < result.time.size(); ++sample) {
        const std::uint64_t target = ((sample * base_steps + 39) / 40) * refinement;
        while (run.metrics()->stamp.epoch < target) {
            report = run.Step();
            if (report.status != CouponStatus::Ok) {
                ADD_FAILURE() << "refinement " << refinement << " epoch " << run.metrics()->stamp.epoch
                              << ": " << report.diagnostic; return false;
            }
        }
        Frame frame;
        report = run.Capture(frame);
        if (report.status != CouponStatus::Ok) { ADD_FAILURE() << report.diagnostic; return false; }
        result.time[sample] = frame.stamp.time; result.tip[sample] = Tip(frame); result.energy[sample] = Energy(frame);
        if (sample && !result.crossing_time && result.tip[sample] <= 0 && result.tip[sample-1] > 0)
            result.crossing_time = result.time[sample-1] + (result.time[sample] - result.time[sample-1]) *
                                  result.tip[sample-1] / (result.tip[sample-1] - result.tip[sample]);
    }
    result.maximum_energy_error = run.metrics()->maximum_relative_energy_error;
    result.minimum_time = result.time[std::min_element(result.tip.begin(), result.tip.end()) - result.tip.begin()];
    EXPECT_EQ(run.metrics()->stamp.epoch, run.metrics()->required_steps);
    EXPECT_NEAR(run.metrics()->stamp.time, run.modal()->horizon, 1e-10);
    EXPECT_EQ(run.metrics()->last_operator_epoch, run.metrics()->required_steps);
    EXPECT_EQ(run.Step().status, CouponStatus::InvalidInput);
    ExpectAllocations(run, state, element);
    return true;
}

// Explicit opt-in after the guarded 100-step cost probe. This executes three
// complete CUDA trajectories, never prescribed motion or a host replacement.
TEST_F(ElasticCoupon, DISABLED_HalfPeriodRefinement) {
    std::array<Trajectory, 3> history;
    for (unsigned level = 0; level < history.size(); ++level) {
        ASSERT_TRUE(RunHalfPeriod(1u << level, history[level]));
        const auto& trajectory = history[level];
        EXPECT_LE(trajectory.maximum_energy_error, .01);
        EXPECT_GT(trajectory.crossing_time, 0);
        EXPECT_LT(trajectory.crossing_time, trajectory.time.back());
        EXPECT_LT(trajectory.tip.back(), -.001);  // Actual deformation reverses sign with visible amplitude.
        EXPECT_GE(trajectory.minimum_time, .8 * trajectory.time.back());
        RecordProperty("energy_error_h_div_" + std::to_string(1u << level), Precise(trajectory.maximum_energy_error));
    }
    for (unsigned level = 0; level < 2; ++level) {
        const auto& coarse = history[level]; const auto& fine = history[level+1];
        for (std::size_t sample = 0; sample < coarse.time.size(); ++sample) {
            EXPECT_NEAR(coarse.time[sample], fine.time[sample], 1e-10);
            // Fixed 1 micrometre floor avoids dividing by a zero-crossing tip.
            EXPECT_NEAR(coarse.tip[sample], fine.tip[sample], std::max(1e-6, .05 * std::abs(fine.tip[sample])));
            EXPECT_NEAR(coarse.energy[sample] / coarse.initial_energy,
                        fine.energy[sample] / fine.initial_energy, .05);
        }
        EXPECT_NEAR(coarse.crossing_time, fine.crossing_time, .05 * fine.time.back());
        EXPECT_NEAR(coarse.minimum_time, fine.minimum_time, .05 * fine.time.back());
        EXPECT_NEAR(coarse.maximum_energy_error, fine.maximum_energy_error, .05);
    }
}
}  // namespace
