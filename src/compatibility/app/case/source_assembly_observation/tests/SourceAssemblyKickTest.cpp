#include "ObservationFixture.h"

namespace crash::cases::source_assembly_observation::test {
TEST(SourceAssemblyObservation, HalfKickUsesCompleteAppliedAndReactionForceAndCouple) {
    Fixture f; const auto n=f.LastOrdinary(); const auto& m=f.bindings.shells().nodes()[n].native;
    f.next.v[3*n]=8.125; f.next.w[3*n+2]=.25;
    const double dt=.5*H;
    f.force[3*n]=m.mass*.125/dt; f.couple[3*n+2]=m.isotropic_inertia*.25/dt;
    // Nonzero supplied reactions cancel only part of each applied load.
    f.reaction[3*n]=-.25*f.force[3*n]; f.force[3*n]-=f.reaction[3*n];
    f.reaction_couple[3*n+2]=-.5*f.couple[3*n+2]; f.couple[3*n+2]-=f.reaction_couple[3*n+2];
    Summary out; ASSERT_TRUE(ObserveInterval(f.input(),&out));
    const long double expected=.5L*m.mass*(8.125L*8.125L-64)+.5L*m.isotropic_inertia*.25L*.25L;
    Near(out.native_delta,expected); Near(out.effective_delta,expected);
    EXPECT_LT(out.reaction.translation,0); EXPECT_LT(out.reaction.rotation,0);
    EXPECT_LE(std::abs(out.native_residual),out.roundoff_budget);
    EXPECT_EQ(out.replacement_delta,0);
}
TEST(SourceAssemblyObservation, FinalOrdinaryAndGroupedReactionCorruptionRollsBackAndRetries) {
    Fixture f; Summary out; ASSERT_TRUE(ObserveInterval(f.input(),&out)); const auto accepted=Bytes(out);
    for(auto n:{f.LastOrdinary(),f.bindings.rigid_groups()->members()[75].global_node}) {
        f.reaction_couple[3*n+2]=1;
        auto result=ObserveInterval(f.input(),&out);
        EXPECT_EQ(result.status,Status::KickMismatch); EXPECT_EQ(result.node,n); EXPECT_EQ(result.dof,5u);
        EXPECT_EQ(Bytes(out),accepted); f.reaction_couple[3*n+2]=0;
        ASSERT_TRUE(ObserveInterval(f.input(),&out)); EXPECT_EQ(Bytes(out),accepted);
    }
}
TEST(SourceAssemblyObservation, ZeroAverageReversalAndWrongKickPhaseCannotHide) {
    Fixture f; const auto n=f.LastOrdinary(); f.next.v[3*n]=-8;
    Summary out; out.effective_delta=97; const auto old=Bytes(out);
    EXPECT_EQ(ObserveInterval(f.input(),&out).status,Status::KickMismatch); EXPECT_EQ(Bytes(out),old);
    f.next.v[3*n]=8; auto input=f.input(); input.prepared.kick_dt=H;
    EXPECT_EQ(ObserveInterval(input,&out).status,Status::InvalidPhase); EXPECT_EQ(Bytes(out),old);
    input=f.input(); input.prepared.base_velocity_time=H;
    EXPECT_EQ(ObserveInterval(input,&out).status,Status::InvalidPhase); EXPECT_EQ(Bytes(out),old);
}
TEST(SourceAssemblyObservation, SubUlpImpulseIsAdmittedButLateOverflowPreservesOutput) {
    Fixture f; const auto n=f.LastOrdinary(); const auto& m=f.bindings.shells().nodes()[n].native;
    f.force[3*n]=m.mass*std::ldexp(1.,-55)/(.5*H);
    Summary out; ASSERT_TRUE(ObserveInterval(f.input(),&out));
    EXPECT_EQ(out.native_delta,0); EXPECT_GT(out.applied.total,0);
    EXPECT_LE(std::abs(out.native_residual),out.roundoff_budget);
    const auto before=Bytes(out); f.reaction.back()=std::numeric_limits<double>::infinity();
    EXPECT_EQ(ObserveInterval(f.input(),&out).status,Status::NonfiniteResult); EXPECT_EQ(Bytes(out),before);
}
TEST(SourceAssemblyObservation, LargeOrdinaryWorkCannotEraseSmallSuppliedGroupReplacement) {
    Fixture f;
    // Supplied phase-metric values, not a claimed dynamically valid trajectory.
    // Aggregate motion and physical member motion need not give identical K;
    // their difference is an explicit diagnostic, never an integration proof.
    f.next_groups[0].state.omega={0,0,.25};
    Summary reference; ASSERT_TRUE(ObserveInterval(f.input(),&reference));
    ASSERT_GT(reference.replacement_delta,0);
    const auto n=f.LastOrdinary(); const auto& m=f.bindings.shells().nodes()[n].native;
    f.next.v[3*n]=1e8;
    f.force[3*n]=m.mass*(f.next.v[3*n]-f.old.v[3*n])/(.5*H);
    Summary large; ASSERT_TRUE(ObserveInterval(f.input(),&large));
    EXPECT_GT(large.native_delta,1e10);
    EXPECT_EQ(large.replacement_delta,reference.replacement_delta);
    EXPECT_EQ(large.after.groups.rotation,reference.after.groups.rotation);
}
TEST(SourceAssemblyObservation, CompleteAcceptedReactionScheduleIsRequired) {
    Fixture f; Summary out; out.native_delta=103; const auto saved=Bytes(out);
    auto input=f.input(); input.base.reaction_kick_dt=std::numeric_limits<double>::quiet_NaN();
    EXPECT_EQ(ObserveInterval(input,&out).status,Status::InvalidPhase); EXPECT_EQ(Bytes(out),saved);
    input=f.input(); input.base.reaction_time=H;
    EXPECT_EQ(ObserveInterval(input,&out).status,Status::InvalidPhase); EXPECT_EQ(Bytes(out),saved);
    f.stamp.epoch=1; f.stamp.time=H; f.stamp.velocity_time=.5*H;
    f.stamp.velocity_phase=fe::NodalVelocityPhase::PreviousMidpoint; f.stamp.reactions_valid=true;
    f.stamp.reaction_kick_dt=H;
    EXPECT_EQ(ObserveInterval(f.input(),&out).status,Status::InvalidPhase); EXPECT_EQ(Bytes(out),saved);
    f.stamp.reaction_kick_dt=.5*H; ASSERT_TRUE(ObserveInterval(f.input(),&out));
}
} // namespace crash::cases::source_assembly_observation::test
