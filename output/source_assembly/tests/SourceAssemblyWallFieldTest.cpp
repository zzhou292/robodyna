#include "SourceAssemblyWallFieldTestSupport.h"
#include <limits>

namespace crash::output::assembly::test {
TEST(SourceAssemblyWallFields, CompleteSyntheticFramePreservesActualSourceAndNativeStressScope) {
    WallFields f;const auto initial=wall_fields::FrameDocument(f.View());
    EXPECT_STREQ(initial["schema"].GetString(),WallFrameSchema);EXPECT_STREQ(initial["kind"].GetString(),WallArtifactKind);
    EXPECT_STREQ(initial["stress_frame"].GetString(),"native_corotational_shell_axes");EXPECT_TRUE(initial["contact"].IsNull());
    EXPECT_TRUE(initial["diagnostics"]["motion"]["before"].IsNull());
    EXPECT_EQ(initial["nodal_fields"]["position_xyz_m"].Size(),3090u);EXPECT_EQ(initial["nodal_fields"]["orientation_wxyz"].Size(),4120u);
    EXPECT_EQ(initial["sections"]["source_parents"].Size(),915u);EXPECT_EQ(initial["sections"]["triangle_binding"].Size(),1719u);
    for(const auto& p:f.surface.parents()) {
        const auto& row=initial["sections"]["source_parents"][p.source_index];EXPECT_EQ(row[1u].GetUint64(),p.element);
        EXPECT_EQ(row[3u].GetUint64(),p.material);EXPECT_EQ(row[4u].GetUint64(),p.section);EXPECT_EQ(row[5u].GetUint64(),p.curve);
        const auto& points=initial["sections"]["sections"][p.source_index][9u];ASSERT_EQ(points.Size(),3u);
        EXPECT_DOUBLE_EQ(points[2u][4u].GetDouble(),-(100*(1+double(p.source_index))+20+4+.5));
    }
    f.Interval();const auto later=wall_fields::FrameDocument(f.View());
    EXPECT_EQ(later["contact"]["nodes"].Size(),1030u);EXPECT_EQ(later["contact"]["parents"].Size(),915u);
    EXPECT_EQ(later["contact"]["nodes"][1029u][1u].GetUint64(),f.bindings.source().data().nodes[1029].source_id);
    EXPECT_STREQ(later["diagnostics"]["motion"]["after"]["phase"]["kind"].GetString(),"stored_midpoint_with_lagged_frame");
    EXPECT_DOUBLE_EQ(later["stamp"]["reaction_kick_dt"].GetDouble(),.5*f.stamp.fixed_dt);
    RecordProperty("synthetic_complete_frame_bytes",wall_files::JsonBytes(later,WallFieldBytes));
}
TEST(SourceAssemblyWallFields, ExtentsStampsMaterialPhaseAndLateContactIdentityRejectWithoutSourceMutation) {
    WallFields f;const auto x=f.x;auto view=f.View();view.nodes.node_count=1;view.nodes.position_xyz=reinterpret_cast<const double*>(1);
    EXPECT_THROW(wall_fields::FrameDocument(view),std::runtime_error);
    view=f.View();view.t3.count=1;view.qeph.values=reinterpret_cast<const fe::ShellBatchSectionState*>(1);
    EXPECT_THROW(wall_fields::FrameDocument(view),std::runtime_error);
    f.Interval();const auto stamp=f.stamp;++f.diagnostics.stamp.rigid_groups.source_instance_id;
    EXPECT_THROW(wall_fields::FrameDocument(f.View()),std::runtime_error);f.diagnostics.stamp=stamp;
    f.captured.qeph.internal_work[0]=1;EXPECT_THROW(wall_fields::FrameDocument(f.View()),std::runtime_error);f.captured=f.diagnostics.shells;
    f.diagnostics.motion.after.phase.frame_time=f.stamp.time;EXPECT_THROW(wall_fields::FrameDocument(f.View()),std::runtime_error);
    f.diagnostics.motion.after.phase.frame_time=f.stamp.reaction_time;
    ++f.contact_parents.back().parent_element_id;EXPECT_THROW(wall_fields::FrameDocument(f.View()),std::runtime_error);--f.contact_parents.back().parent_element_id;
    const auto face=f.faces.back();f.faces.back()=UINT64_MAX;EXPECT_THROW(wall_fields::FrameDocument(f.View()),std::runtime_error);f.faces.back()=face;
    f.sections.t.back().history.point[2].stress[4]=std::numeric_limits<double>::quiet_NaN();EXPECT_THROW(wall_fields::FrameDocument(f.View()),std::runtime_error);
    EXPECT_EQ(f.x,x);
}
TEST(SourceAssemblyWallFields, ConfigurationRetainsAllMaterialCurvesGroupsAndReleasedFrontier) {
    WallFields f;const auto config=wall_fields::ConfigurationDocument(f.bindings,f.setup,Configuration(),f.surface,Request());
    const auto& input=config["input"];EXPECT_EQ(input["materials"].Size(),6u);EXPECT_EQ(input["sections"].Size(),6u);
    EXPECT_EQ(input["curves"].Size(),2u);EXPECT_EQ(input["native_nodes"].Size(),1030u);EXPECT_EQ(input["internal_rigid_groups"].Size(),6u);
    EXPECT_EQ(input["boundary"]["released_nodal_rigid_ids"].Size(),4u);EXPECT_EQ(input["boundary"]["released_spotweld_ids"].Size(),13u);
    std::size_t members=0;for(const auto& g:input["internal_rigid_groups"].GetArray())members+=g["members"].Size();EXPECT_EQ(members,76u);
    EXPECT_EQ(config["wall_setup"]["nodal_area_m2"].Size(),1030u);EXPECT_EQ(config["wall_setup"]["contact_parents"].Size(),915u);
    EXPECT_EQ(config["wall_setup"]["group_initial_kinetic"].Size(),6u);
    EXPECT_STREQ(config["source_inventory_sha256"].GetString(),f.bindings.source().data().identity.sha256.c_str());
    EXPECT_DOUBLE_EQ(config["wall_setup"]["native_initial_kinetic_J"][0u].GetDouble(),f.setup.certificate()->initial_kinetic.native_nodes.value);
    EXPECT_DOUBLE_EQ(config["wall_setup"]["aggregate_initial_kinetic_J"][0u].GetDouble(),f.setup.certificate()->initial_kinetic.with_aggregate_groups.value);
    RecordProperty("actual_configuration_bytes",wall_files::JsonBytes(config,WallConfigurationBytes));
}
TEST(SourceAssemblyWallFields, IntervalRowsRequireSameActualScopeAndNeverInventInitialContact) {
    WallFields f;const auto base=f.stamp;EXPECT_THROW(wall_fields::IntervalRow(base,f.diagnostics,f.Contact()),std::runtime_error);
    f.Interval();const auto row=wall_fields::IntervalRow(base,f.diagnostics,f.Contact());
    EXPECT_EQ(std::count(row.begin(),row.end(),','),WallIntervalColumns-1);EXPECT_EQ(row.back(),'\n');
    auto wrong=base;++wrong.rigid_groups.member_count;EXPECT_THROW(wall_fields::IntervalRow(wrong,f.diagnostics,f.Contact()),std::runtime_error);
    ++f.c.attempt;EXPECT_THROW(wall_fields::IntervalRow(base,f.diagnostics,f.Contact()),std::runtime_error);
}
} // namespace crash::output::assembly::test
