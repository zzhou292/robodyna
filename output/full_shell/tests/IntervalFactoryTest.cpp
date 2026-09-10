#include "TestSupport.h"
#include "output/source_assembly/SourceAssemblyWallFields.h"
#include <iomanip>
#include <sstream>

namespace crash::output::full_shell::test {
namespace wall=output::assembly::wall_fields;
namespace dynamics=cases::source_assembly_dynamics;
namespace fe=tl::fea;
struct FactoryValues {
    FactoryValues() {
        base.owner_id=UINT64_C(9007199254740993);base.node_count=1;base.fixed_dt=.125;base.has_rotations=true;
        base.temporal_scheme=fe::NodalTemporalScheme::StaggeredHalfKickStart;
        d.stamp=base;auto& s=d.stamp;s.epoch=1;s.time=.125;s.velocity_time=.0625;
        s.velocity_phase=fe::NodalVelocityPhase::PreviousMidpoint;s.reactions_valid=true;s.reaction_kick_dt=.0625;
        d.has_interval=true;d.shells.valid=true;
        auto family=[&](auto& f) {
            f.valid=true;f.owner_id=base.owner_id;f.epoch=1;f.base_epoch=0;f.time=s.time;
            f.velocity_time=s.velocity_time;f.kick_dt=s.reaction_kick_dt;
            f.configuration_id=17;f.qualification_id=19;f.attempt=UINT64_C(18014398509481986);
            f.phase=decltype(f.phase)::Accepted;f.has_completed_interval=true;
        };
        family(d.shells.qeph);family(d.shells.t3);
        c.valid=true;c.owner_id=base.owner_id;c.time=s.time;c.velocity_time=s.velocity_time;c.kick_dt=s.reaction_kick_dt;
        c.configuration_id=17;c.qualification_id=19;c.attempt=d.shells.qeph.attempt;
        c.phase=tlfea::contact::NodalWallDevicePhase::PreparedCandidate;
        d.motion.after.native_total=3;d.motion.after.effective_total=4;
        d.native_internal_work=-5;d.cumulative_plastic_work=6;
        d.motion.native_delta=-7;d.motion.effective_delta=8;d.motion.replacement_delta=-9;
        d.motion.applied.total=10;d.motion.reaction.total=-11;d.motion.native_residual=12;
        d.motion.effective_residual=-13;d.motion.roundoff_budget=14;
        c.resultant.value=15;c.resultant.error=16;c.potential.value=17;c.potential.error=18;
        c.kick_work=-19;c.drift_work=20;c.work_uncertainty=21;c.quadratic_work_upper=22;c.conservative_defect=-23;
        c.wall_kick_impulse=24;c.wall_kick_impulse_error=25;c.maximum_penetration=26;
        d.maximum_plastic_strain=28;d.maximum_rotation=31;d.maximum_area_ratio=32;d.maximum_thickness_ratio=33;
    }
    dynamics::ContactView View() const {return {&c,&parent,&node,&face,1,1};}
    fe::NodalStamp base;
    dynamics::Diagnostics d;
    tlfea::contact::NodalWallDiagnostics c;
    tlfea::contact::NodalWallParentResult parent;
    tlfea::contact::NodalWallPointResult node;
    std::uint64_t face=1;
};
TEST(IntervalFactory,SharedTypedFactoryPreservesLegacyColumnBytesAndSignedValues) {
    FactoryValues f;
    const auto typed=wall::IntervalValues(f.base,f.d,f.View());
    const double expected[]{.0625,.0625,3,4,-5,6,-7,8,-9,10,-11,12,-13,14,15,16,17,18,-19,20,21,22,-23,24,25,26,0,28,0,0,31,32,33};
    std::ostringstream legacy;
    legacy<<std::setprecision(17)<<f.base.owner_id<<",0,"<<f.c.attempt<<",0,1,0.125";
    for(double v:expected)legacy<<','<<v;
    legacy<<'\n';
    EXPECT_EQ(wall::IntervalRow(f.base,f.d,f.View()),legacy.str());
    EXPECT_EQ(interval::CsvRow(typed),legacy.str());
    ASSERT_EQ(typed.reals.size(),35u);
    for(std::size_t i=0;i<33;++i)EXPECT_EQ(Bits(typed.reals[i+2]),Bits(expected[i]));
    EXPECT_EQ(typed.integers[0],UINT64_C(9007199254740993));
    EXPECT_EQ(typed.integers[2],UINT64_C(18014398509481986));
    EXPECT_STREQ(assembly::WallIntervalHeader,interval::CsvHeader);
}
TEST(IntervalFactory,WrongMaterialContactAndCommonPhaseRejectBeforeSerialization) {
    for(int fault=0;fault<9;++fault) {
        FactoryValues f;
        if(fault==0)++f.d.shells.qeph.owner_id;
        if(fault==1)f.d.shells.t3.time=.25;
        if(fault==2)f.d.shells.qeph.base_velocity_time=.125;
        if(fault==3)++f.d.shells.t3.configuration_id;
        if(fault==4)++f.d.shells.qeph.qualification_id;
        if(fault==5)f.d.shells.t3.valid=false;
        if(fault==6)++f.c.attempt;
        if(fault==7)f.c.base_time=.125;
        if(fault==8)f.d.has_interval=false;
        EXPECT_THROW(wall::IntervalValues(f.base,f.d,f.View()),std::exception)<<fault;
        EXPECT_THROW(wall::IntervalRow(f.base,f.d,f.View()),std::exception)<<fault;
    }
}
TEST(IntervalFactory,LegacyArchiveRejectsUnrepresentedV2MaterialSchema) {
    EXPECT_NO_THROW(assembly::CheckWallSourceSchema("robo-dyna.source-assembly-inventory.v1"));
    EXPECT_THROW(assembly::CheckWallSourceSchema("robo-dyna.source-assembly-inventory.v2"),std::exception);
    EXPECT_THROW(assembly::CheckWallSourceSchema(""),std::exception);
}
} // namespace crash::output::full_shell::test
