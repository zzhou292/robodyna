#include "QephSpinFixture.h"

namespace crash::cases::source_assembly_observation::test {
TEST(SourceAssemblyQephSpin,ActualCompleteIncidenceAndInitialUncomputedKinematics) {
    SpinFixture f;QephSpinObservation out;ASSERT_TRUE(ObserveQephSpin(f.spin_input(),&out));
    EXPECT_EQ(out.global_node,459);ASSERT_EQ(out.parent_count,2);
    EXPECT_EQ(out.parents[0].source_parent,2214871);EXPECT_EQ(out.parents[1].source_parent,2214872);
    EXPECT_EQ(out.parents[0].local_node,1);EXPECT_EQ(out.parents[1].local_node,0);
    EXPECT_TRUE(out.candidate_parents[0].has_native_kinematics);
    EXPECT_FALSE(out.parents[0].has_native_kinematics);EXPECT_FALSE(out.completed);EXPECT_EQ(out.enclosing.owner_id,0);
    EXPECT_EQ(out.native.isotropic_inertia,f.bindings.shells().nodes()[459].native.isotropic_inertia);
    EXPECT_FALSE(CheckQephSpinSource(f.bindings,0));EXPECT_FALSE(CheckQephSpinSource(f.bindings,UINT64_MAX));
    EXPECT_FALSE(CheckQephSpinSource(f.bindings,f.bindings.rigid_groups()->members()[0].source_node_id));
    const auto t=f.bindings.shells().t3_nodes(0)[0];EXPECT_FALSE(CheckQephSpinSource(f.bindings,f.bindings.shells().nodes()[t].source_id));
}
TEST(SourceAssemblyQephSpin,RetainsActualPacketsAndDistinctBasePhaseWithoutForceReevaluation) {
    SpinFixture f;f.Later();QephSpinObservation out;ASSERT_TRUE(ObserveQephSpin(f.spin_input(),&out));
    EXPECT_EQ(out.base.epoch,2);EXPECT_EQ(out.base.time,2*H);EXPECT_EQ(out.base.velocity_time,1.5*H);
    EXPECT_EQ(out.attempt,9);EXPECT_FALSE(out.completed);
    EXPECT_EQ(out.assembly_couple_residual.x,0);EXPECT_EQ(out.assembly_couple_residual.y,0);EXPECT_EQ(out.assembly_couple_residual.z,0);
    for(unsigned i=0;i<2;++i) {
        const auto& p=out.parents[i];EXPECT_TRUE(p.has_native_kinematics);const auto& original=f.parents[392+i];
        EXPECT_EQ(p.force.proposed_history.data().stabilization[11],original.proposed_history.data().stabilization[11]);
        EXPECT_EQ(p.force.proposed_history.data().material_stress[0],original.proposed_history.data().material_stress[0]);
        EXPECT_EQ(p.section.history.point[2].stress[4],f.sections[392+i].history.point[2].stress[4]);
        EXPECT_EQ(p.section.history.point[2].plastic_strain,f.sections[392+i].history.point[2].plastic_strain);
        EXPECT_EQ(out.candidate_parents[i].force.internal_couple[0].x,f.candidate_parents[392+i].internal_couple[0].x);
        EXPECT_EQ(out.candidate_parents[i].section.history.point[2].plastic_strain,f.candidate_sections[392+i].history.point[2].plastic_strain);
        EXPECT_NE(out.candidate_parents[i].section.history.point[2].plastic_strain,p.section.history.point[2].plastic_strain);
        const auto n=p.native_normal,w=out.omega,c=original.internal_couple[p.local_node];
        const long double projected=static_cast<long double>(w.x)*n.x+static_cast<long double>(w.y)*n.y+static_cast<long double>(w.z)*n.z;
        Near(p.normal_spin,projected);Near(p.normal_internal_couple,static_cast<long double>(c.x)*n.x+static_cast<long double>(c.y)*n.y+static_cast<long double>(c.z)*n.z);
    }
}
TEST(SourceAssemblyQephSpin,WrongPhaseCountAndLateInvalidInputsPreserveOutputAndRetry) {
    SpinFixture f;f.Later();QephSpinObservation out;ASSERT_TRUE(ObserveQephSpin(f.spin_input(),&out));const auto bytes=Bytes(out);
    auto in=f.spin_input();in.parent_count--;in.parents=reinterpret_cast<const fe::qeph::ForceTrial*>(1);
    EXPECT_FALSE(ObserveQephSpin(in,&out));EXPECT_EQ(Bytes(out),bytes);
    in=f.spin_input();in.prepared.base_time+=H;EXPECT_FALSE(ObserveQephSpin(in,&out));EXPECT_EQ(Bytes(out),bytes);
    in=f.spin_input();in.configuration_id++;EXPECT_FALSE(ObserveQephSpin(in,&out));EXPECT_EQ(Bytes(out),bytes);
    in=f.spin_input();f.diagnostics.epoch++;EXPECT_FALSE(ObserveQephSpin(in,&out));f.UpdateDiagnostics();
    const auto saved=f.parents[393];f.parents[393].kinematics.hourglass_rate[5]=std::numeric_limits<double>::quiet_NaN();
    EXPECT_FALSE(ObserveQephSpin(f.spin_input(),&out));EXPECT_EQ(Bytes(out),bytes);f.parents[393]=saved;
    const auto cd=f.candidate_diagnostics;f.candidate_diagnostics.attempt++;
    EXPECT_FALSE(ObserveQephSpin(f.spin_input(),&out));EXPECT_EQ(Bytes(out),bytes);f.candidate_diagnostics=cd;
    const auto ch=f.candidate_parents[393];f.candidate_parents[393].kinematics.regular_rate[7]=std::numeric_limits<double>::quiet_NaN();
    EXPECT_FALSE(ObserveQephSpin(f.spin_input(),&out));EXPECT_EQ(Bytes(out),bytes);f.candidate_parents[393]=ch;
    f.couple[3*459]=std::numeric_limits<double>::infinity();EXPECT_FALSE(ObserveQephSpin(f.spin_input(),&out));EXPECT_EQ(Bytes(out),bytes);
    f.couple[3*459]=out.applied_couple.x;ASSERT_TRUE(ObserveQephSpin(f.spin_input(),&out));
    EXPECT_EQ(out.assembly_couple_residual.x,0);const auto retry=Bytes(out);
    in=f.spin_input();in.applied_couple_xyz=reinterpret_cast<const double*>(&out);
    EXPECT_FALSE(ObserveQephSpin(in,&out));EXPECT_EQ(Bytes(out),retry);
}
}
