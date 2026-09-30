#include "ObservationFixture.h"

namespace crash::cases::source_assembly_observation::test {
TEST(SourceAssemblyObservation, ActualInitialPartitionCountsOrdinaryAndSixGroupsOnce) {
    Fixture f; KineticSummary out;
    ASSERT_TRUE(ObserveInitial(f.bindings,f.stamp,f.old.view(),f.old_groups.data(),6,Kinetic(f.bindings,f.old),&out));
    EXPECT_EQ(out.phase.kind,rigid::ObservationPhaseKind::PhysicalInitialization);
    long double mass=0,primary=0,group_mass=0;
    for(std::size_t n=0;n<f.stamp.node_count;++n) mass+=f.bindings.shells().nodes()[n].native.mass;
    for(std::size_t g=0;g<6;++g) {
        primary+=f.bindings.rigid_groups()->groups()[g].regularization.primary_mass_kg;
        group_mass+=f.bindings.rigid_groups()->groups()[g].structural_mass_kg;
    }
    Near(out.native_total,32*mass); Near(out.effective_total,32*(mass+primary));
    Near(out.ordinary.total,32*(mass-group_mass)); Near(out.grouped_members.total,32*group_mass);
    Near(out.groups.primary_translation,32*primary); EXPECT_GT(out.groups.primary_translation,0);
    EXPECT_EQ(out.groups.rotation,0); EXPECT_EQ(out.ordinary.added_rotation,0);
}
TEST(SourceAssemblyObservation, FullTensorAndNativeScalarInertiaRemainDifferentPartitions) {
    Fixture f; f.stamp.epoch=1; f.stamp.time=H; f.stamp.velocity_time=.5*H;
    f.stamp.velocity_phase=fe::NodalVelocityPhase::PreviousMidpoint;
    f.stamp.reactions_valid=true; f.stamp.reaction_kick_dt=.5*H;
    // Supplied-value mapping fixture, not a claimed owner trajectory.
    for(std::size_t g=0;g<6;++g) f.old_groups[g].state.omega=f.next_groups[g].state.omega={.5,-.25,.125};
    for(std::size_t n=0;n<f.stamp.node_count;++n) {
        f.old.w[3*n]=f.next.w[3*n]=.5; f.old.w[3*n+1]=f.next.w[3*n+1]=-.25;
        f.old.w[3*n+2]=f.next.w[3*n+2]=.125;
    }
    Summary out; ASSERT_TRUE(ObserveInterval(f.input(),&out));
    long double group_rotation=0;
    for(std::size_t g=0;g<6;++g) {
        const auto& tensor=f.bindings.rigid_groups()->groups()[g].effective_tensor;
        const long double w[]{.5,-.25,.125};
        for(unsigned a=0;a<3;++a) for(unsigned b=0;b<3;++b) group_rotation+=.5L*w[a]*tensor.v[3*a+b]*w[b];
    }
    Near(out.after.groups.rotation,group_rotation);
    Near(out.after.grouped_members.native_rotation,
         out.after.grouped_members.physical_rotation+out.after.grouped_members.added_rotation);
    EXPECT_GT(out.after.groups.member_orbital_rotation,0);
    EXPECT_GT(out.after.groups.native_member_rotation,0);
    EXPECT_GT(out.after.grouped_members.added_rotation,out.after.grouped_members.physical_rotation);
    EXPECT_EQ(out.native_delta,0); EXPECT_EQ(out.effective_delta,0);
}
TEST(SourceAssemblyObservation, WrongFinalGroupAndPublicationCannotPartiallyPublish) {
    Fixture f; Summary out; out.effective_delta=73; const auto before=Bytes(out);
    auto input=f.input(); ++f.next_groups.back().source_node_set_id;
    EXPECT_EQ(ObserveInterval(input,&out).status,Status::WrongIdentity); EXPECT_EQ(Bytes(out),before);
    --f.next_groups.back().source_node_set_id; input.kinetic.translation+=1;
    EXPECT_EQ(ObserveInterval(input,&out).status,Status::KineticMismatch); EXPECT_EQ(Bytes(out),before);
    input=f.input(); f.next.w.back()=std::numeric_limits<double>::infinity();
    EXPECT_FALSE(ObserveInterval(input,&out)); EXPECT_EQ(Bytes(out),before);
}
TEST(SourceAssemblyObservation, MissingRangesWrongSourceAndOverlappingOutputRejectBeforeReads) {
    Fixture f; Summary out; out.effective_delta=81; const auto before=Bytes(out);
    auto input=f.input(); input.before.velocity_xyz=nullptr;
    EXPECT_EQ(ObserveInterval(input,&out).status,Status::InvalidInput); EXPECT_EQ(Bytes(out),before);
    input=f.input(); ++input.prepared.rigid_groups.source_instance_id;
    EXPECT_EQ(ObserveInterval(input,&out).status,Status::WrongIdentity); EXPECT_EQ(Bytes(out),before);
    input=f.input(); input.reaction_force_xyz=reinterpret_cast<const double*>(&out);
    EXPECT_EQ(ObserveInterval(input,&out).status,Status::InvalidInput); EXPECT_EQ(Bytes(out),before);
    input=f.input(); input.group_count=SIZE_MAX;
    EXPECT_EQ(ObserveInterval(input,&out).status,Status::InvalidInput); EXPECT_EQ(Bytes(out),before);
}
} // namespace crash::cases::source_assembly_observation::test
