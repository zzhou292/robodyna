#include "output/source_assembly/QephSpinTrace.h"
#include "case/source_assembly_observation/tests/QephSpinFixture.h"
#include <limits>

namespace crash::output::assembly::test {
namespace observer=cases::source_assembly_observation;
observer::QephSpinObservation ValueRecord() {
    observer::test::SpinFixture f;f.Later();observer::QephSpinObservation v;
    const auto r=observer::ObserveQephSpin(f.spin_input(),&v);Require(bool(r),r.message);
    // Pure serializer fixture only. The production path obtains this full stamp
    // from the actual owner after the common commit, never from this arithmetic.
    v.enclosing=v.base;++v.enclosing.epoch;v.enclosing.time=v.base.time+v.base.fixed_dt;
    v.enclosing.velocity_time=v.base.time+.5*v.base.fixed_dt;v.enclosing.reactions_valid=true;
    v.enclosing.reaction_base_epoch=v.base.epoch;v.enclosing.reaction_time=v.base.time;
    v.enclosing.reaction_kick_dt=v.base.fixed_dt;v.completed=true;return v;
}
TEST(QephSpinFields, CompleteNativePacketSourceUnitsAndForcePhaseAreExplicit) {
    const auto v=ValueRecord();const auto d=QephSpinDocument(v);
    EXPECT_EQ(d["source_node_id"].GetUint64(),2181592u);EXPECT_EQ(d["complete_incident_qeph_parent_count"].GetUint64(),2u);
    EXPECT_EQ(d["base_stamp"]["epoch"].GetUint64(),2u);EXPECT_EQ(d["enclosing_accepted_stamp"]["epoch"].GetUint64(),3u);
    ASSERT_EQ(d["parents"].Size(),2u);const auto& p=d["parents"][1u];
    EXPECT_EQ(p["source_element_id"].GetUint64(),2214872u);EXPECT_EQ(p["source_material_id"].GetUint64(),2000145u);
    EXPECT_EQ(p["retained_history"]["HOURG_native_mixed_units"][11u].GetDouble(),v.parents[1].force.proposed_history.data().stabilization[11]);
    EXPECT_EQ(p["retained_kinematics"]["origin_base_time_s"].GetDouble(),v.base.reaction_time);
    EXPECT_EQ(p["points"][2u][5u].GetDouble(),v.parents[1].section.history.point[2].plastic_strain);
    const auto& candidate=d["enclosing_candidate_parents"][1u];
    EXPECT_EQ(candidate["retained_history"]["epoch"].GetUint64(),3u);
    EXPECT_EQ(candidate["retained_kinematics"]["origin_base_time_s"].GetDouble(),v.base.time);
    EXPECT_EQ(candidate["points"][2u][5u].GetDouble(),v.candidate_parents[1].section.history.point[2].plastic_strain);
    EXPECT_FALSE(p.HasMember("actual_assembled_couple_N_m")); // Assembly load belongs to the node, not each parent.
}
TEST(QephSpinFields, InvalidPhaseLateNonfiniteAndForecastFailClosed) {
    auto v=ValueRecord();v.completed=false;
    EXPECT_THROW(QephSpinDocument(v),std::runtime_error);v.completed=true;
    ++v.enclosing.rigid_groups.member_count;
    EXPECT_THROW(QephSpinDocument(v),std::runtime_error);--v.enclosing.rigid_groups.member_count;
    v.parents[1].force.diagnostics.hourglass_viscous_work_increment=std::numeric_limits<double>::quiet_NaN();
    EXPECT_THROW(QephSpinDocument(v),std::runtime_error);
    EXPECT_EQ(PlanQephSpinTrace(8192,8),1028u*SpinTraceRowCap);
    EXPECT_THROW(PlanQephSpinTrace(0,1),std::runtime_error);
    EXPECT_THROW(PlanQephSpinTrace(8,0),std::runtime_error);
    EXPECT_THROW(PlanQephSpinTrace(8,9),std::runtime_error);
    EXPECT_THROW(PlanQephSpinTrace(8192,1),std::runtime_error);
}
}
