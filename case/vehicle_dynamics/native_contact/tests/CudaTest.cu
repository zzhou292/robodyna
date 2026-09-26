#include "Fixture.h"
namespace native_contribution_test {

TEST(NativeContributionCuda, DirectParityIncludesActiveCommonRejectionAndRetry) {
    Rig wrapped;
    nr::Rig direct;
    ASSERT_NO_THROW(wrapped.Initialize());
    ASSERT_NO_THROW(direct.Initialize());
    ASSERT_NO_FATAL_FAILURE(Same(wrapped.Read(), direct.Read(), false));
    const auto resources = wrapped.contribution->resources();
    EXPECT_GT(resources.device_bytes, 0u);
    EXPECT_GT(resources.host_bytes, 0u);
    const auto* stable = &wrapped.contribution->transaction();
    std::uint64_t active = 0;
    for (unsigned step = 0; step < 470; ++step) {
        SCOPED_TRACE(step);
        if (step == 468) {
            const auto before = wrapped.Read();
            Attempt rejected;
            ASSERT_NO_THROW(wrapped.Assemble(rejected));
            ASSERT_GT(wrapped.observation.diagnostics.active_forces, 0u);
            ASSERT_NO_THROW(wrapped.PrepareMaterials(rejected));
            ASSERT_NO_THROW(wrapped.Seal(rejected));
            const auto receipt = *wrapped.contribution->scratch_receipts().self_contact;
            EXPECT_NE(wrapped.physical.Commit(rejected, false).status, fe::ShellPublicationStatus::Success);
            wrapped.Discard();
            EXPECT_EQ(wrapped.contribution->scratch_receipts().self_contact, nullptr);
            ASSERT_NO_FATAL_FAILURE(Same(before, wrapped.Read()));
            // A copied genuine receipt remains descriptive but its issuer
            // generation/attempt has been revoked by the common rejection.
            EXPECT_NE(wrapped.physical.publication->SealPhysicalScratchParticipation(
                wrapped.physical.owner, rejected.token, {nullptr, &receipt}).status,
                fe::ShellPublicationStatus::Success);
            wrapped.Discard();
        }
        Attempt a, b;
        ASSERT_NO_THROW(wrapped.Assemble(a));
        ASSERT_NO_THROW(direct.BeginMaterials(b));
        ASSERT_NO_THROW(Check(direct.contact.AssembleAccepted(direct.owner, b.token, b.assembly)));
        EXPECT_EQ(wrapped.observation.force_base.epoch, step);
        EXPECT_EQ(wrapped.observation.attempt, a.assembly.attempt);
        EXPECT_EQ(wrapped.observation.source.source_id, 1u);
        ASSERT_NO_FATAL_FAILURE(Same(wrapped.observation.diagnostics, direct.contact.last_diagnostics()));
        active += wrapped.observation.diagnostics.active_forces;
        if (step >= 467) {
            std::array<double, 54> x{}, y{};
            std::array<double, 18> kx{}, ky{};
            ASSERT_NO_THROW(wrapped.physical.Force(a, x, kx));
            ASSERT_NO_THROW(direct.Force(b, y, ky));
            Bits(x, y); Bits(kx, ky);
        }
        ASSERT_NO_THROW(wrapped.PrepareMaterials(a));
        ASSERT_NO_THROW(wrapped.Seal(a));
        ASSERT_NE(wrapped.contribution->scratch_receipts().self_contact, nullptr);
        EXPECT_TRUE(wrapped.contribution->scratch_receipts().self_contact->valid());
        ASSERT_NO_THROW(direct.Prepare(b));
        ASSERT_NO_THROW(Check(wrapped.physical.Commit(a)));
        wrapped.contribution->Committed();
        ASSERT_NO_THROW(Check(direct.Commit(b)));
        EXPECT_EQ(wrapped.contribution->scratch_receipts().self_contact, nullptr);
        EXPECT_EQ(&wrapped.contribution->transaction(), stable);
        if (step >= 467) {
            const auto accepted = wrapped.Read();
            EXPECT_EQ(accepted.stamp.epoch, step + 1);
            EXPECT_EQ(accepted.contact.force_base_stamp.epoch, step);
            ASSERT_NO_FATAL_FAILURE(Same(accepted, direct.Read(), false));
        }
        if (HasFailure()) return;
    }
    EXPECT_GT(active, 0u);
    EXPECT_EQ(wrapped.contribution->resources().device_bytes, resources.device_bytes);
    EXPECT_EQ(wrapped.contribution->resources().host_bytes, resources.host_bytes);
}

TEST(NativeContributionCuda, LocalProtocolFailuresPreserveObservationAcceptedStateAndRetry) {
    Rig rig;
    ASSERT_NO_THROW(rig.Initialize());
    const auto initial = rig.Read();
    rig.observation = Sentinel();
    Attempt absent;
    EXPECT_THROW(rig.contribution->SealCandidate(rig.physical.owner, absent.token,
        absent.prepared, absent.common, rig.observation), app::StageError);
    Unchanged(rig.observation);
    rig.Discard();
    ASSERT_NO_FATAL_FAILURE(Same(initial, rig.Read()));

    Attempt duplicate;
    ASSERT_NO_THROW(rig.Assemble(duplicate));
    rig.observation = Sentinel();
    EXPECT_THROW(rig.contribution->Assemble(rig.physical.owner, duplicate.token,
        duplicate.assembly, rig.observation), app::StageError);
    Unchanged(rig.observation);
    EXPECT_EQ(rig.contribution->scratch_receipts().self_contact, nullptr);
    rig.Discard();
    ASSERT_NO_FATAL_FAILURE(Same(initial, rig.Read()));

    Attempt mismatch;
    ASSERT_NO_THROW(rig.Assemble(mismatch));
    ASSERT_NO_THROW(rig.PrepareMaterials(mismatch));
    auto wrong = mismatch.prepared;
    ++wrong.attempt;
    rig.observation = Sentinel();
    EXPECT_THROW(rig.contribution->SealCandidate(rig.physical.owner, mismatch.token,
        wrong, mismatch.common, rig.observation), app::StageError);
    Unchanged(rig.observation);
    rig.Discard();
    ASSERT_NO_FATAL_FAILURE(Same(initial, rig.Read()));
    ASSERT_NO_THROW(rig.Step());
    EXPECT_EQ(rig.Read().stamp.epoch, 1u);
}

TEST(NativeContributionCuda, RealRosterAndReceiptRemainMandatoryAndDestructionUnbinds) {
    Rig rig;
    ASSERT_NO_THROW(rig.Initialize(false));
    Attempt unbound;
    rig.observation = Sentinel();
    ASSERT_NO_THROW(rig.physical.BeginMaterials(unbound));
    EXPECT_THROW(rig.contribution->Assemble(rig.physical.owner, unbound.token,
        unbound.assembly, rig.observation), app::StageError);
    Unchanged(rig.observation);
    rig.Discard();
    ASSERT_NO_THROW(rig.Bind());
    const auto initial = rig.Read();
    Attempt missing;
    ASSERT_NO_THROW(rig.Assemble(missing));
    ASSERT_NO_THROW(rig.PrepareMaterials(missing));
    ASSERT_NO_THROW(rig.contribution->SealCandidate(rig.physical.owner, missing.token,
        missing.prepared, missing.common, rig.observation));
    EXPECT_NE(rig.physical.publication->SealPhysicalScratchParticipation(
        rig.physical.owner, missing.token, {}).status, fe::ShellPublicationStatus::Success);
    rig.Discard();
    ASSERT_NO_FATAL_FAILURE(Same(initial, rig.Read()));
    ASSERT_NO_THROW(rig.Step());
    const auto accepted = rig.Read();
    rig.contribution.reset(); // Concrete issuer detaches from the live publisher.
    Attempt no_contributor;
    ASSERT_NO_THROW(rig.physical.BeginMaterials(no_contributor));
    ASSERT_NO_THROW(rig.PrepareMaterials(no_contributor));
    EXPECT_NE(rig.physical.publication->SealPhysicalScratchParticipation(
        rig.physical.owner, no_contributor.token, {}).status, fe::ShellPublicationStatus::Success);
    rig.physical.publication->DiscardTrial();
    rig.physical.owner.Discard();
    EXPECT_TRUE(fe::trial_identity::SameStamp(accepted.stamp, rig.physical.owner.accepted()));
}

TEST(NativeContributionCuda, ForeignOwnerAndDetachedPublisherFailuresRetainNativeStatus) {
    Rig rig;
    nr::Rig other;
    ASSERT_NO_THROW(rig.Initialize());
    ASSERT_NO_THROW(other.Initialize());
    const auto before = rig.Read();
    Attempt a;
    ASSERT_NO_THROW(other.BeginMaterials(a));
    rig.observation = Sentinel();
    try {
        rig.contribution->Assemble(other.owner, a.token, a.assembly, rig.observation);
        FAIL() << "Foreign owner unexpectedly admitted";
    } catch (const app::StageError& error) {
        EXPECT_EQ(error.failure().operation, app::Operation::Assemble);
        EXPECT_EQ(error.failure().status, n::TransactionStatus::OwnerFailure);
    }
    Unchanged(rig.observation);
    other.owner.Discard();
    other.publication->DiscardTrial();
    other.quad.DiscardTrial();
    other.triangle.DiscardTrial();
    rig.Discard();
    ASSERT_NO_FATAL_FAILURE(Same(before, rig.Read()));
    ASSERT_NO_THROW(rig.Step());
    rig.physical.publication.reset();
    EXPECT_FALSE(rig.contribution->transaction().accepted().available);
    Attempt detached;
    ASSERT_NO_THROW(rig.physical.BeginMaterials(detached));
    rig.observation = Sentinel();
    try {
        rig.contribution->Assemble(rig.physical.owner, detached.token, detached.assembly, rig.observation);
        FAIL() << "Detached publisher unexpectedly admitted";
    } catch (const app::StageError& error) {
        EXPECT_EQ(error.failure().status, n::TransactionStatus::PublicationFailure);
    }
    Unchanged(rig.observation);
    rig.contribution->Discard();
    EXPECT_EQ(rig.physical.owner.accepted().epoch, 1u);
}
} // namespace native_contribution_test
