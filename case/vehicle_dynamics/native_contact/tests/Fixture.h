#pragma once
#include "case/vehicle_dynamics/native_contact/Contribution.h"
#include "lib_utest/qualification/radioss_type25_runtime/MovingSceneRig.h"
#include "lib_utest/qualification/radioss_type25_local_geometry/Assertions.h"
#include <cstring>

namespace native_contribution_test {
namespace app = crash::cases::vehicle_dynamics::native_contact;
namespace nr = native_runtime_test;
namespace fe = tl::fea;
namespace n = app::native;
using nr::Check;
using nr::Attempt;
using nr::State;

// Reuse the owning numerical fixture. Its source observations remain test-only;
// the wrapper's production target never links this header or captured values.
struct Rig {
    nr::Rig physical;
    std::unique_ptr<app::Contribution> contribution; // Dies before physical owner/publisher.
    app::Observation observation;

    void Initialize(bool bind = true) {
        physical.Initialize(nr::ObservedSource::Limits(), false);
        auto transaction = std::make_unique<n::Transaction>();
        Check(transaction->Initialize(nr::ObservedSource::Config(), physical.source.View(),
            physical.owner, *physical.publication, physical.fixture.physical,
            physical.Participants(), physical.Identity(), nr::ObservedSource::Limits()));
        contribution = app::Contribution::Adopt(std::move(transaction));
        if (bind) Bind();
    }
    void Bind() {
        Check(physical.publication->ConfigurePhysicalScratchParticipation(physical.owner,
            physical.fixture.physical, physical.Participants(), physical.Identity(),
            {{}, contribution->roster_entry()}));
    }
    void Assemble(Attempt& a) {
        physical.BeginMaterials(a);
        contribution->Assemble(physical.owner, a.token, a.assembly, observation);
    }
    void PrepareMaterials(Attempt& a) {
        Check(physical.owner.SealAssembly(a.token));
        Check(fe::AdvanceStaggeredCin(physical.owner, a.token,
            {a.assembly.owner_id, a.assembly.accepted.base_epoch, a.assembly.attempt,
             nodal_empty_test::Fixture::Qualification, physical.fixture.fixed_dt, .2, true,
             {fe::NodalCinStructuralProfile::NativeOrdinaryRigidTrace, .8, true}}));
        Check(physical.owner.BorrowPrepared(a.token, &a.prepared));
        Check(physical.quad.EvaluateCandidate(physical.owner, a.token, a.prepared, &a.material.qeph));
        Check(physical.triangle.EvaluateCandidate(physical.owner, a.token, a.prepared, &a.material.t3));
        Check(physical.publication->PreparePhysical(physical.owner, a.token,
            {&a.material.qeph, &a.material.t3}, &a.common));
    }
    void Seal(Attempt& a) {
        contribution->SealCandidate(physical.owner, a.token, a.prepared, a.common, observation);
        Check(physical.publication->SealPhysicalScratchParticipation(
            physical.owner, a.token, contribution->scratch_receipts()));
    }
    void Step() {
        Attempt a;
        Assemble(a);
        PrepareMaterials(a);
        Seal(a);
        Check(physical.Commit(a));
        contribution->Committed();
    }
    void Discard() {
        contribution->Discard();
        physical.quad.DiscardTrial();
        physical.triangle.DiscardTrial();
        physical.publication->DiscardTrial();
        physical.owner.Discard();
    }
    State Read() {
        State result;
        Check(physical.owner.CopyAccepted({result.x.data(), result.v.data(), 18,
            result.q.data(), result.omega.data(), result.reaction.data(), result.couple.data()}, &result.stamp));
        double numerical = 0;
        fe::NodalStamp stamp;
        Check(physical.owner.CopyAcceptedCin({result.mass.data(), result.inertia.data(),
            nullptr, nullptr, &numerical, 18, 0}, &stamp));
        if (numerical != 0) throw std::runtime_error("Unexpected numerical mass");
        Check(contribution->transaction().CopyAccepted(
            {result.history.data(), result.initial_contact.data(), 18}, &result.contact));
        return result;
    }
};
template<class T, std::size_t N>
void Bits(const std::array<T, N>& a, const std::array<T, N>& b) {
    EXPECT_EQ(std::memcmp(a.data(), b.data(), N * sizeof(T)), 0);
}
inline void Same(const State& a, const State& b, bool same_owner = true) {
    if (same_owner) EXPECT_TRUE(fe::trial_identity::SameStamp(a.stamp, b.stamp));
    EXPECT_EQ(a.stamp.epoch, b.stamp.epoch);
    EXPECT_EQ(a.stamp.time, b.stamp.time);
    EXPECT_EQ(a.stamp.velocity_phase, b.stamp.velocity_phase);
    EXPECT_EQ(a.stamp.reaction_base_epoch, b.stamp.reaction_base_epoch);
    EXPECT_EQ(a.stamp.reaction_kick_dt, b.stamp.reaction_kick_dt);
    Bits(a.x, b.x); Bits(a.v, b.v); Bits(a.q, b.q); Bits(a.omega, b.omega);
    Bits(a.mass, b.mass); Bits(a.inertia, b.inertia);
    Bits(a.reaction, b.reaction); Bits(a.couple, b.couple);
    EXPECT_EQ(a.initial_contact, b.initial_contact);
    EXPECT_EQ(a.contact.available, b.contact.available);
    EXPECT_EQ(a.contact.stamp.epoch, b.contact.stamp.epoch);
    EXPECT_EQ(a.contact.force_phase_available, b.contact.force_phase_available);
    EXPECT_EQ(a.contact.force_base_stamp.epoch, b.contact.force_base_stamp.epoch);
    EXPECT_EQ(a.contact.generation, b.contact.generation);
    EXPECT_EQ(a.contact.selectors.history, b.contact.selectors.history);
    EXPECT_EQ(a.contact.selectors.reference, b.contact.selectors.reference);
    EXPECT_EQ(a.contact.selectors.reference_generation, b.contact.selectors.reference_generation);
    EXPECT_EQ(a.contact.selectors.has_reference, b.contact.selectors.has_reference);
    for (unsigned i = 0; i < 18; ++i) {
        SCOPED_TRACE(i);
        type25_geometry_test::Same(a.history[i], b.history[i], true);
    }
}
inline void Same(const n::TransactionDiagnostics& a, const n::TransactionDiagnostics& b) {
    EXPECT_EQ(a.raw_candidates, b.raw_candidates);
    EXPECT_EQ(a.optimized_candidates, b.optimized_candidates);
    EXPECT_EQ(a.kept_occurrences, b.kept_occurrences);
    EXPECT_EQ(a.active_forces, b.active_forces);
    EXPECT_EQ(a.reference_rebuilt, b.reference_rebuilt);
    Bits(std::array<double, 3>{a.elastic_energy, a.damping_work, a.friction_work},
         std::array<double, 3>{b.elastic_energy, b.damping_work, b.friction_work});
}
inline app::Observation Sentinel() {
    app::Observation result;
    result.enabled = true;
    result.force_base.owner_id = 8987;
    result.attempt = 9011;
    result.source = {9123, 22, 23, 24, 25, 26, 27, true};
    result.diagnostics = {14, 15, 16, 17, true, 1.5, 2.5, 3.5};
    return result;
}
inline void Unchanged(const app::Observation& result) {
    const auto expected = Sentinel();
    EXPECT_EQ(result.enabled, expected.enabled);
    EXPECT_TRUE(fe::trial_identity::SameStamp(result.force_base, expected.force_base));
    EXPECT_EQ(result.attempt, expected.attempt);
    EXPECT_EQ(result.source.source_id, expected.source.source_id);
    EXPECT_EQ(result.source.topology_generation, expected.source.topology_generation);
    EXPECT_EQ(result.source.source_generation, expected.source.source_generation);
    EXPECT_EQ(result.source.nodes, expected.source.nodes);
    EXPECT_EQ(result.source.secondaries, expected.source.secondaries);
    EXPECT_EQ(result.source.primary_mains, expected.source.primary_mains);
    EXPECT_EQ(result.source.expanded_mains, expected.source.expanded_mains);
    EXPECT_EQ(result.source.available, expected.source.available);
    Same(result.diagnostics, expected.diagnostics);
}
} // namespace native_contribution_test
