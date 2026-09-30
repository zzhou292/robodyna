#include "ElasticCouponModel.h"
#include "lib_src/elements/ReissnerShellBatch.h"
#include "lib_src/solvers/ExplicitNodalStep.h"
#include "chrono/core/ChQuaternion.h"
#include "chrono/core/ChVector3.h"
#include <cuda_runtime.h>
#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstring>

namespace crash::qualification {
namespace {
namespace fe = tl::fea;
namespace shell = tl::fea::reissner;
using reference::ElasticCouponConfiguration;
using reference::ElasticCouponModel;
using reference::ElasticCouponStatus;
using Results = std::array<shell::ShellResult, reference::kCouponElements>;
constexpr std::uint64_t kConfiguration = 0xB20001;
constexpr double kStep = 1e-8;

template <class T> std::array<unsigned char, sizeof(T)> Bytes(const T& value) {
    std::array<unsigned char, sizeof(T)> bytes;
    std::memcpy(bytes.data(), &value, sizeof(T));
    return bytes;
}

struct Input {
    std::array<double, 18> x{}, v{}, omega{};
    std::array<double, 24> q{};
    std::array<double, 6> inverse_mass{}, inverse_inertia{};
    std::array<std::uint8_t, 6> translation_fixed{}, rotation_fixed{};

    explicit Input(const ElasticCouponModel& model, bool loaded = true) {
        auto configuration = model.data().reference_configuration;
        inverse_mass = model.data().inverse_mass;
        inverse_inertia = model.data().inverse_isotropic_inertia;
        for (unsigned n = 0; n < 6; ++n) {
            const bool fixed = model.data().fixed[n];
            translation_fixed[n] = fixed ? 7 : 0;
            rotation_fixed[n] = fixed;
            if (fixed || !loaded) continue;
            configuration.position[n].x += 2e-6 * (n + 1);
            configuration.position[n].y -= 1e-6 * (n + 1);
            configuration.position[n].z += 5e-6 * (n + 1);
            const double half_angle = .00025 * (n + 1), s = std::sin(half_angle) / std::sqrt(6.);
            configuration.rotation[n] = {std::cos(half_angle), s, 2*s, -s};
            v[3*n] = .01 * (n + 1); v[3*n+1] = -.02; v[3*n+2] = .015;
            omega[3*n] = -.3; omega[3*n+1] = .2 * (n + 1); omega[3*n+2] = .4;
        }
        SetConfiguration(configuration);
    }

    void SetConfiguration(const ElasticCouponConfiguration& configuration) {
        for (unsigned n = 0; n < 6; ++n) {
            const auto& p = configuration.position[n]; const auto& r = configuration.rotation[n];
            x[3*n] = p.x; x[3*n+1] = p.y; x[3*n+2] = p.z;
            q[4*n] = r.w; q[4*n+1] = r.x; q[4*n+2] = r.y; q[4*n+3] = r.z;
        }
    }
    ElasticCouponConfiguration Configuration() const {
        ElasticCouponConfiguration result;
        for (unsigned n = 0; n < 6; ++n) {
            result.position[n] = {x[3*n], x[3*n+1], x[3*n+2]};
            result.rotation[n] = {q[4*n], q[4*n+1], q[4*n+2], q[4*n+3]};
        }
        return result;
    }
};

struct Snapshot {
    std::array<double, 18> x{}, v{}, omega{}, reaction_force{}, reaction_couple{};
    std::array<double, 24> q{};
    fe::NodalStamp stamp;
};
struct Trial {
    fe::NodalTrialToken token;
    fe::NodalAssemblyView assembly;
    fe::NodalPreparedView prepared;
    shell::ShellBatchDiagnostics base, candidate;
};
struct DeviceRotations {
    double* values = nullptr;
    ~DeviceRotations() { if (values) cudaFree(values); }
};

__global__ void InvalidBatchLaunchInjection() {}

void CopyForces(const fe::NodalAssemblyView& view, std::array<double,36>& output) {
    const double* channels[] = {view.forces.force_x,view.forces.force_y,view.forces.force_z,
                               view.forces.couple_x,view.forces.couple_y,view.forces.couple_z};
    for (unsigned c=0;c<6;++c)
        ASSERT_EQ(cudaMemcpyAsync(output.data()+6*c,channels[c],6*sizeof(double),
                                  cudaMemcpyDeviceToHost,view.stream),cudaSuccess);
    ASSERT_EQ(cudaStreamSynchronize(view.stream),cudaSuccess);
}

void Near(double actual, double expected) {
    EXPECT_NEAR(actual, expected, 1e-10 * (1 + std::abs(expected)));
}
void SameSnapshot(const Snapshot& a, const Snapshot& b, bool same_owner = true) {
    EXPECT_EQ(a.x, b.x); EXPECT_EQ(a.v, b.v); EXPECT_EQ(a.q, b.q); EXPECT_EQ(a.omega, b.omega);
    EXPECT_EQ(a.reaction_force, b.reaction_force); EXPECT_EQ(a.reaction_couple, b.reaction_couple);
    if (same_owner) EXPECT_EQ(a.stamp.owner_id, b.stamp.owner_id);
    EXPECT_EQ(a.stamp.epoch, b.stamp.epoch); EXPECT_EQ(a.stamp.time, b.stamp.time);
    EXPECT_EQ(a.stamp.fixed_dt, b.stamp.fixed_dt); EXPECT_EQ(a.stamp.node_count, b.stamp.node_count);
    EXPECT_EQ(a.stamp.has_rotations, b.stamp.has_rotations);
    EXPECT_EQ(a.stamp.reactions_valid, b.stamp.reactions_valid);
    EXPECT_EQ(a.stamp.reaction_base_epoch, b.stamp.reaction_base_epoch);
    EXPECT_EQ(a.stamp.reaction_time, b.stamp.reaction_time);
}

class ReissnerShellBatchCuda : public ::testing::Test {
  protected:
    ElasticCouponModel model;

    void SetUp() override {
        int count = 0;
        ASSERT_EQ(cudaGetDeviceCount(&count), cudaSuccess);
        ASSERT_GT(count, 0) << "This qualification requires an actual CUDA device";
    }
    void Initialize(fe::FENodalState& owner, shell::ReissnerShellBatch& batch, const Input& in) {
        fe::NodalStateConfig config; config.node_count = 6; config.fixed_dt = kStep;
        const fe::HostNodalKinematicsView state{in.x.data(), in.v.data(), in.omega.data(), 6, in.q.data()};
        const fe::NodalDofConfig dofs{in.translation_fixed.data(), in.rotation_fixed.data(), in.inverse_inertia.data()};
        const auto initialized = owner.Initialize(config, state, in.inverse_mass.data(), dofs);
        ASSERT_EQ(initialized.status, fe::NodalStatus::Ok) << initialized.message;
        std::array<shell::ReissnerShellBatchElement, 2> elements;
        for (unsigned e = 0; e < 2; ++e) {
            elements[e].reference = model.data().reference[e]; elements[e].section = model.data().section[e];
            for (unsigned n = 0; n < 4; ++n) elements[e].nodes[n] = model.data().connectivity[e][n];
        }
        shell::ReissnerShellBatchConfig batch_config;
        batch_config.owner = owner.accepted(); batch_config.configuration_id = kConfiguration;
        batch_config.element_count = 2;
        batch_config.drilling_policy = shell::ShellDrillingInertiaPolicy::kEqualPhysicalTangential;
        const auto initialized_batch = batch.Initialize(batch_config, elements.data());
        ASSERT_EQ(initialized_batch.status, shell::ShellBatchStatus::kSuccess) << initialized_batch.message;
        EXPECT_EQ(batch.allocations().device_allocations, 1);
        EXPECT_LE(batch.allocations().device_bytes, 1024*1024);
    }
    void Begin(fe::FENodalState& owner, shell::ReissnerShellBatch& batch, Trial& trial) {
        ASSERT_EQ(owner.BeginTrial(&trial.token, &trial.assembly).status, fe::NodalStatus::Ok);
        const auto assembled = batch.Assemble(trial.assembly, &trial.base);
        ASSERT_EQ(assembled.status, shell::ShellBatchStatus::kSuccess) << assembled.message;
    }
    void Prepare(fe::FENodalState& owner, shell::ReissnerShellBatch& batch, Trial& trial) {
        ASSERT_NO_FATAL_FAILURE(Begin(owner, batch, trial));
        ASSERT_EQ(owner.SealAssembly(trial.token).status, fe::NodalStatus::Ok);
        // Test-only two-step transaction exercise, far below the declared rate
        // bound. Physical trajectory/envelope qualification belongs to the case.
        fe::NodalStepAdmission admission;
        admission.owner_id = trial.assembly.owner_id; admission.base_epoch = trial.base.base_epoch;
        admission.attempt = trial.assembly.attempt; admission.maximum_dt = kStep;
        admission.maximum_rotation_increment = .01;
        admission.kind = fe::NodalStepAdmissionKind::RestrictedElasticTrajectory;
        admission.qualification_id = kConfiguration; admission.stiffness_rate_envelope = 1e12;
        const auto advanced = fe::AdvanceNodal(owner, trial.token, admission);
        ASSERT_EQ(advanced.status, fe::NodalStatus::Ok) << advanced.message;
        ASSERT_EQ(owner.BorrowPrepared(trial.token, &trial.prepared).status, fe::NodalStatus::Ok);
        const auto evaluated = batch.EvaluateCandidate(trial.prepared, &trial.candidate);
        ASSERT_EQ(evaluated.status, shell::ShellBatchStatus::kSuccess) << evaluated.message;
    }
    void Commit(fe::FENodalState& owner, const Trial& trial) {
        const auto& value = trial.candidate;
        ASSERT_TRUE(value.valid);
        ASSERT_EQ(value.owner_id, owner.accepted().owner_id);
        ASSERT_EQ(value.base_epoch, owner.accepted().epoch);
        ASSERT_EQ(value.attempt, trial.prepared.attempt);
        ASSERT_EQ(value.configuration_id, kConfiguration);
        ASSERT_EQ(value.phase, shell::ShellBatchPhase::kPreparedCandidate);
        ASSERT_TRUE(std::isfinite(value.elastic_energy));
        ASSERT_TRUE(std::isfinite(value.conservative_force_coordinate_defect));
        ASSERT_NEAR(value.kinetic_work_residual, 0, 1e-14);
        ASSERT_LT(value.maximum_pair_angle, .01);
        ASSERT_GT(value.minimum_signed_area_ratio, .99);
        ASSERT_GT(value.minimum_display_triangle_area_ratio, .99);
        const fe::NodalValidationReceipt receipt{value.owner_id, value.base_epoch, value.attempt, kConfiguration, true};
        ASSERT_EQ(fe::CompleteNodalValidation(owner, trial.token, receipt).status, fe::NodalStatus::Ok);
        ASSERT_EQ(owner.Commit(trial.token).status, fe::NodalStatus::Ok);
    }
    void Read(fe::FENodalState& owner, Snapshot& output) {
        const fe::NodalSnapshotBuffer buffer{output.x.data(), output.v.data(), 6, output.q.data(),
            output.omega.data(), output.reaction_force.data(), output.reaction_couple.data()};
        ASSERT_EQ(owner.CopyAccepted(buffer, &output.stamp).status, fe::NodalStatus::Ok);
    }
    void ExpectChrono(shell::ReissnerShellBatch& batch, const shell::ShellBatchDiagnostics& value,
                      const ElasticCouponConfiguration& configuration) {
        Results actual;
        ASSERT_EQ(batch.CopyElementResults(value, actual.data(), actual.size()).status, shell::ShellBatchStatus::kSuccess);
        reference::ElasticCouponEvaluation expected; std::string diagnostic;
        ASSERT_EQ(model.EvaluateChrono(configuration, expected, diagnostic), ElasticCouponStatus::kSuccess) << diagnostic;
        Near(value.elastic_energy, expected.energy); Near(value.bending_energy, expected.bending_energy);
        for (unsigned e = 0; e < 2; ++e) {
            SCOPED_TRACE(e);
            Near(actual[e].energy, expected.element[e].energy);
            Near(actual[e].bending_energy, expected.element[e].bending_energy);
            for (unsigned n = 0; n < 4; ++n) {
                const auto& a = actual[e]; const auto& b = expected.element[e];
                Near(a.force[n].x, b.force[n].x); Near(a.force[n].y, b.force[n].y); Near(a.force[n].z, b.force[n].z);
                Near(a.couple[n].x, b.couple[n].x); Near(a.couple[n].y, b.couple[n].y); Near(a.couple[n].z, b.couple[n].z);
                for (unsigned c = 0; c < 12; ++c) {
                    Near(a.strain[n][c], b.strain[n][c]); Near(a.resultant[n][c], b.resultant[n][c]);
                }
            }
        }
    }
};

TEST_F(ReissnerShellBatchCuda, SharedNodeKineticLedgerUsesGlobalMassAndWorldDirectors) {
    Input in(model, false);
    const double s = std::sin(.35)/std::sqrt(14.);
    const chrono::ChQuaterniond rotation(std::cos(.35), s, 2*s, -3*s);
    auto configuration = in.Configuration();
    for (unsigned n = 0; n < 6; ++n) {
        const auto& x = configuration.position[n];
        const auto rotated = rotation.Rotate(chrono::ChVector3d(x.x, x.y, x.z));
        configuration.position[n] = {rotated.x(), rotated.y(), rotated.z()};
        configuration.rotation[n] = {rotation.e0(), rotation.e1(), rotation.e2(), rotation.e3()};
        if (in.rotation_fixed[n]) continue;
        in.v[3*n] = .1*(n+1); in.v[3*n+1] = -.03*n; in.v[3*n+2] = .07;
        in.omega[3*n] = .7; in.omega[3*n+1] = -.2*(n+1); in.omega[3*n+2] = 1.1;
    }
    in.SetConfiguration(configuration);
    fe::FENodalState owner; shell::ReissnerShellBatch batch; Trial trial;
    ASSERT_NO_FATAL_FAILURE(Initialize(owner, batch, in));
    ASSERT_NO_FATAL_FAILURE(Begin(owner, batch, trial));
    // Independent rectangular row-sum ledger, each SHARED node visited once.
    // m=rho*t*area/4; J=m*t^2/12. Physical rank two plus drilling rank one
    // gives J*I, not (J+J)*I. No element mass/energy helper is used here.
    constexpr std::array<double,6> mass{{.1,.05,.05,.1,.05,.05}};
    const auto director = rotation.Rotate(chrono::ChVector3d(0, 0, 1));
    double translation = 0, physical = 0, drilling = 0;
    for (unsigned n = 0; n < 6; ++n) {
        const chrono::ChVector3d v(in.v[3*n], in.v[3*n+1], in.v[3*n+2]);
        const chrono::ChVector3d w(in.omega[3*n], in.omega[3*n+1], in.omega[3*n+2]);
        const double J = mass[n]*.02*.02/12, axial = w.Dot(director);
        translation += .5*mass[n]*v.Length2();
        physical += .5*J*(w.Length2()-axial*axial); drilling += .5*J*axial*axial;
    }
    EXPECT_NEAR(trial.base.kinetic_translation, translation, 1e-14);
    EXPECT_NEAR(trial.base.kinetic_physical_rotation, physical, 1e-17);
    EXPECT_NEAR(trial.base.kinetic_artificial_drilling, drilling, 1e-17);
    EXPECT_GT(physical, 0); EXPECT_GT(drilling, 0);
    ASSERT_NO_FATAL_FAILURE(ExpectChrono(batch, trial.base, configuration));
    owner.Discard();
}

TEST_F(ReissnerShellBatchCuda, MismatchedOwnerMassOrInertiaMakesAssemblyFailureSticky) {
    for (bool inertia : {false, true}) {
        SCOPED_TRACE(inertia);
        Input in(model); (inertia ? in.inverse_inertia : in.inverse_mass)[0] *= 2;
        fe::FENodalState owner; shell::ReissnerShellBatch batch; Trial trial;
        ASSERT_NO_FATAL_FAILURE(Initialize(owner, batch, in));
        Snapshot before, after; ASSERT_NO_FATAL_FAILURE(Read(owner, before));
        ASSERT_EQ(owner.BeginTrial(&trial.token, &trial.assembly).status, fe::NodalStatus::Ok);
        trial.base.elastic_energy = -123; const auto sentinel = Bytes(trial.base);
        const auto failed = batch.Assemble(trial.assembly, &trial.base);
        EXPECT_EQ(failed.status, shell::ShellBatchStatus::kInvalidMass); EXPECT_EQ(failed.node, 0);
        EXPECT_EQ(Bytes(trial.base), sentinel);
        EXPECT_EQ(owner.SealAssembly(trial.token).status, fe::NodalStatus::ContributorFailure);
        owner.Discard(); ASSERT_NO_FATAL_FAILURE(Read(owner, after)); SameSnapshot(before, after);
    }
}

TEST_F(ReissnerShellBatchCuda, DuplicateAssemblyInvalidatesAttemptAndFreshAssemblyRetries) {
    Input in(model); fe::FENodalState owner; shell::ReissnerShellBatch batch; Trial trial;
    ASSERT_NO_FATAL_FAILURE(Initialize(owner, batch, in));
    ASSERT_NO_FATAL_FAILURE(Begin(owner, batch, trial));
    const auto sentinel = Bytes(trial.base);
    EXPECT_EQ(batch.Assemble(trial.assembly, &trial.base).status, shell::ShellBatchStatus::kStaleTrial);
    EXPECT_EQ(Bytes(trial.base), sentinel);
    EXPECT_EQ(owner.SealAssembly(trial.token).status, fe::NodalStatus::ContributorFailure);
    owner.Discard();
    Trial retry; ASSERT_NO_FATAL_FAILURE(Begin(owner, batch, retry));
    EXPECT_GT(retry.base.attempt, trial.base.attempt);
    ASSERT_NO_FATAL_FAILURE(ExpectChrono(batch, retry.base, in.Configuration()));
    owner.Discard();
}

TEST_F(ReissnerShellBatchCuda, RecoverableLaunchPoisonPreventsScatterAndKeepsContributorFailureSticky) {
    for (bool detect_during_readback : {false,true}) {
        SCOPED_TRACE(detect_during_readback);
        Input in(model); fe::FENodalState owner; shell::ReissnerShellBatch batch; Trial trial;
        ASSERT_NO_FATAL_FAILURE(Initialize(owner,batch,in));
        Snapshot before,after; ASSERT_NO_FATAL_FAILURE(Read(owner,before));
        if (detect_during_readback) {
            ASSERT_NO_FATAL_FAILURE(Begin(owner,batch,trial));
            Results output{}; output[1].energy=-37; const auto saved=Bytes(output);
            InvalidBatchLaunchInjection<<<1,0,0,trial.assembly.stream>>>();
            const auto pending=cudaPeekAtLastError();
            ASSERT_TRUE(pending==cudaErrorInvalidValue || pending==cudaErrorInvalidConfiguration);
            EXPECT_EQ(batch.CopyElementResults(trial.base,output.data(),output.size()).status,
                      shell::ShellBatchStatus::kDeviceFailure);
            EXPECT_EQ(Bytes(output),saved); EXPECT_EQ(cudaPeekAtLastError(),cudaSuccess);
            owner.Discard();
        }
        ASSERT_EQ(owner.BeginTrial(&trial.token,&trial.assembly).status,fe::NodalStatus::Ok);
        // Preserve an existing participant's finite, nonzero forces/couples.
        std::array<double,36> seeded{},actual{};
        for (unsigned i=0;i<seeded.size();++i) seeded[i]=.125*(i+1);
        double* channels[] = {trial.assembly.forces.force_x,trial.assembly.forces.force_y,
            trial.assembly.forces.force_z,trial.assembly.forces.couple_x,
            trial.assembly.forces.couple_y,trial.assembly.forces.couple_z};
        for (unsigned c=0;c<6;++c)
            ASSERT_EQ(cudaMemcpyAsync(channels[c],seeded.data()+6*c,6*sizeof(double),
                                      cudaMemcpyHostToDevice,trial.assembly.stream),cudaSuccess);
        ASSERT_EQ(cudaStreamSynchronize(trial.assembly.stream),cudaSuccess);
        trial.base.elastic_energy=-113; const auto diagnostic=Bytes(trial.base);
        if (!detect_during_readback) {
            InvalidBatchLaunchInjection<<<1,0,0,trial.assembly.stream>>>();
            const auto pending=cudaPeekAtLastError();
            ASSERT_TRUE(pending==cudaErrorInvalidValue || pending==cudaErrorInvalidConfiguration);
        }
        EXPECT_EQ(batch.Assemble(trial.assembly,&trial.base).status,shell::ShellBatchStatus::kDeviceFailure);
        EXPECT_EQ(cudaPeekAtLastError(),cudaSuccess); EXPECT_EQ(Bytes(trial.base),diagnostic);
        ASSERT_NO_FATAL_FAILURE(CopyForces(trial.assembly,actual)); EXPECT_EQ(actual,seeded);
        EXPECT_EQ(owner.SealAssembly(trial.token).status,fe::NodalStatus::ContributorFailure);
        owner.Discard(); ASSERT_NO_FATAL_FAILURE(Read(owner,after)); SameSnapshot(before,after);
    }
}

TEST_F(ReissnerShellBatchCuda, DeviceElementResultsMatchChronoAndRejectIncorrectOutputIdentity) {
    Input in(model); fe::FENodalState owner; shell::ReissnerShellBatch batch; Trial trial;
    ASSERT_NO_FATAL_FAILURE(Initialize(owner, batch, in));
    ASSERT_NO_FATAL_FAILURE(Begin(owner, batch, trial));
    ASSERT_NO_FATAL_FAILURE(ExpectChrono(batch, trial.base, in.Configuration()));
    Results output{}; output[1].energy = -123;
    const auto sentinel = Bytes(output);
    for (unsigned fault = 0; fault < 6; ++fault) {
        SCOPED_TRACE(fault); auto wrong = trial.base;
        switch (fault) {
            case 0: ++wrong.owner_id; break;
            case 1: ++wrong.base_epoch; break;
            case 2: ++wrong.attempt; break;
            case 3: ++wrong.configuration_id; break;
            case 4: wrong.phase = shell::ShellBatchPhase::kPreparedCandidate; break;
            case 5: wrong.valid = false; break;
        }
        EXPECT_EQ(batch.CopyElementResults(wrong, output.data(), output.size()).status, shell::ShellBatchStatus::kStaleTrial);
        EXPECT_EQ(Bytes(output), sentinel);
    }
    EXPECT_EQ(batch.CopyElementResults(trial.base, output.data(), 1).status, shell::ShellBatchStatus::kResourceLimit);
    EXPECT_EQ(Bytes(output), sentinel);
    ASSERT_NO_FATAL_FAILURE(ExpectChrono(batch, trial.base, in.Configuration()));
    owner.Discard();
}

TEST_F(ReissnerShellBatchCuda, CandidateWorkAndDeviceElementResultsUseThePreparedState) {
    Input in(model); fe::FENodalState owner; shell::ReissnerShellBatch batch; Trial trial;
    ASSERT_NO_FATAL_FAILURE(Initialize(owner, batch, in));
    ASSERT_NO_FATAL_FAILURE(Prepare(owner, batch, trial));
    EXPECT_EQ(trial.candidate.owner_id, trial.base.owner_id);
    EXPECT_EQ(trial.candidate.base_epoch, trial.base.base_epoch);
    EXPECT_EQ(trial.candidate.attempt, trial.base.attempt);
    EXPECT_EQ(trial.candidate.phase, shell::ShellBatchPhase::kPreparedCandidate);
    EXPECT_EQ(trial.candidate.base_elastic_energy, trial.base.elastic_energy);
    EXPECT_EQ(trial.candidate.base_kinetic_energy, trial.base.kinetic_translation +
        trial.base.kinetic_physical_rotation + trial.base.kinetic_artificial_drilling);
    EXPECT_GT(trial.candidate.mass_weighted_increment_squared, 0);
    EXPECT_NEAR(trial.candidate.kinetic_energy_increment, trial.candidate.kinetic_midpoint_work, 1e-14);
    // Explicit test/output cadence readback; production stepping reads scalars.
    Input prepared(model, false);
    ASSERT_EQ(cudaMemcpyAsync(prepared.x.data(), trial.prepared.kinematics.position_xyz, sizeof(prepared.x),
                              cudaMemcpyDeviceToHost, trial.prepared.stream), cudaSuccess);
    ASSERT_EQ(cudaMemcpyAsync(prepared.q.data(), trial.prepared.kinematics.orientation_wxyz, sizeof(prepared.q),
                              cudaMemcpyDeviceToHost, trial.prepared.stream), cudaSuccess);
    ASSERT_EQ(cudaStreamSynchronize(trial.prepared.stream), cudaSuccess);
    ASSERT_NO_FATAL_FAILURE(ExpectChrono(batch, trial.candidate, prepared.Configuration()));
    Results results{}; const auto sentinel = Bytes(results);
    EXPECT_EQ(batch.CopyElementResults(trial.base, results.data(), 2).status, shell::ShellBatchStatus::kStaleTrial);
    EXPECT_EQ(Bytes(results), sentinel);
    ASSERT_NO_FATAL_FAILURE(Commit(owner, trial));
    Snapshot accepted; ASSERT_NO_FATAL_FAILURE(Read(owner, accepted));
    EXPECT_EQ(accepted.x, prepared.x); EXPECT_EQ(accepted.q, prepared.q);
    EXPECT_EQ(accepted.stamp.epoch, 1);
}

TEST_F(ReissnerShellBatchCuda, ForeignAndStaleCandidatesPreserveDiagnosticsAndInvalidateResults) {
    Input in(model); fe::FENodalState owner, foreign_owner;
    shell::ReissnerShellBatch batch, foreign_batch; Trial trial, foreign;
    ASSERT_NO_FATAL_FAILURE(Initialize(owner, batch, in));
    ASSERT_NO_FATAL_FAILURE(Initialize(foreign_owner, foreign_batch, in));
    ASSERT_NO_FATAL_FAILURE(Prepare(owner, batch, trial));
    ASSERT_NO_FATAL_FAILURE(Prepare(foreign_owner, foreign_batch, foreign));
    const auto sentinel = Bytes(trial.candidate);
    Results output{}; output[1].energy = -123; const auto result_sentinel = Bytes(output);
    EXPECT_EQ(batch.EvaluateCandidate(foreign.prepared, &trial.candidate).status, shell::ShellBatchStatus::kWrongOwner);
    EXPECT_EQ(Bytes(trial.candidate), sentinel);
    EXPECT_EQ(batch.CopyElementResults(trial.candidate, output.data(), 2).status, shell::ShellBatchStatus::kStaleTrial);
    EXPECT_EQ(Bytes(output), result_sentinel);
    for (bool epoch : {false, true}) {
        auto stale = trial.prepared;
        if (epoch) ++stale.kinematics.base_epoch; else ++stale.attempt;
        EXPECT_EQ(batch.EvaluateCandidate(stale, &trial.candidate).status, shell::ShellBatchStatus::kStaleTrial);
        EXPECT_EQ(Bytes(trial.candidate), sentinel);
    }
    ASSERT_EQ(batch.EvaluateCandidate(trial.prepared, &trial.candidate).status, shell::ShellBatchStatus::kSuccess);
    ASSERT_EQ(batch.CopyElementResults(trial.candidate, output.data(), 2).status, shell::ShellBatchStatus::kSuccess);
    owner.Discard(); foreign_owner.Discard();
}

TEST_F(ReissnerShellBatchCuda, LateSecondElementChartFailureDiscardsAndRetryMatchesCleanOwner) {
    Input in(model); fe::FENodalState owner, clean_owner;
    shell::ReissnerShellBatch batch, clean_batch; Trial first, clean_first;
    ASSERT_NO_FATAL_FAILURE(Initialize(owner, batch, in));
    ASSERT_NO_FATAL_FAILURE(Initialize(clean_owner, clean_batch, in));
    ASSERT_NO_FATAL_FAILURE(Prepare(owner, batch, first));
    ASSERT_NO_FATAL_FAILURE(Commit(owner, first));
    ASSERT_NO_FATAL_FAILURE(Prepare(clean_owner, clean_batch, clean_first));
    ASSERT_NO_FATAL_FAILURE(Commit(clean_owner, clean_first));
    Snapshot before, after; ASSERT_NO_FATAL_FAILURE(Read(owner, before));
    Trial failed; ASSERT_NO_FATAL_FAILURE(Prepare(owner, batch, failed));
    // Fault injection uses separate read-only input storage. No borrowed owner
    // state is overwritten; node 4 belongs only to the second element.
    DeviceRotations altered;
    ASSERT_EQ(cudaMalloc(reinterpret_cast<void**>(&altered.values), sizeof(in.q)), cudaSuccess);
    ASSERT_EQ(cudaMemcpyAsync(altered.values, failed.prepared.kinematics.orientation_wxyz, sizeof(in.q),
                              cudaMemcpyDeviceToDevice, failed.prepared.stream), cudaSuccess);
    const std::array<double,4> outside{{std::cos(1.1), std::sin(1.1), 0, 0}};
    ASSERT_EQ(cudaMemcpyAsync(altered.values + 4*4, outside.data(), sizeof(outside),
                              cudaMemcpyHostToDevice, failed.prepared.stream), cudaSuccess);
    auto invalid = failed.prepared; invalid.kinematics.orientation_wxyz = altered.values;
    const auto diagnostic_sentinel = Bytes(failed.candidate);
    const auto report = batch.EvaluateCandidate(invalid, &failed.candidate);
    EXPECT_EQ(report.status, shell::ShellBatchStatus::kElementFailure);
    EXPECT_EQ(report.element, 1); EXPECT_EQ(report.element_status, shell::ShellStatus::kOutsideChart);
    EXPECT_EQ(Bytes(failed.candidate), diagnostic_sentinel);
    Results output{}; output[0].energy = -123; const auto result_sentinel = Bytes(output);
    EXPECT_EQ(batch.CopyElementResults(failed.candidate, output.data(), 2).status, shell::ShellBatchStatus::kStaleTrial);
    EXPECT_EQ(Bytes(output), result_sentinel);
    owner.Discard();
    EXPECT_NE(owner.Commit(failed.token).status, fe::NodalStatus::Ok);
    ASSERT_NO_FATAL_FAILURE(Read(owner, after)); SameSnapshot(before, after);
    Trial retry, clean_second;
    ASSERT_NO_FATAL_FAILURE(Prepare(owner, batch, retry));
    ASSERT_NO_FATAL_FAILURE(Commit(owner, retry));
    ASSERT_NO_FATAL_FAILURE(Prepare(clean_owner, clean_batch, clean_second));
    ASSERT_NO_FATAL_FAILURE(Commit(clean_owner, clean_second));
    Snapshot retried, clean; ASSERT_NO_FATAL_FAILURE(Read(owner, retried));
    ASSERT_NO_FATAL_FAILURE(Read(clean_owner, clean)); SameSnapshot(retried, clean, false);
    EXPECT_GT(retry.candidate.attempt, failed.candidate.attempt);
    EXPECT_EQ(retry.candidate.elastic_energy, clean_second.candidate.elastic_energy);
    EXPECT_EQ(retry.candidate.kinetic_midpoint_work, clean_second.candidate.kinetic_midpoint_work);
}
}  // namespace
}  // namespace crash::qualification
