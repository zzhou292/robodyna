#include "Support.h"
namespace crash::cases::vehicle_startup::physical_model::supports_test {
TEST(VehicleSupportsPhysicalOriginal, All44OriginalJointsHaveBothActualBodiesAndRequiredAxes) {
    const auto& source=JointSource(); const auto& value=Joints();
    ASSERT_EQ(source.policy(),JointPolicy); ASSERT_EQ(source.data().rows.size(),44u);
    EXPECT_EQ(source.data().required,44u); EXPECT_EQ(source.data().boundaries,0u);
    ASSERT_EQ(value.model().joints().size(),44u); ASSERT_EQ(value.source_rows().size(),44u);
    EXPECT_TRUE(value.model().domain()->SharesStorage(Domain().domain()));
    unsigned rods=0;
    for (std::size_t i=0;i<value.source_rows().size();++i) {
        const auto& row=source.data().rows[value.source_rows()[i]];
        const auto& joint=value.model().joints()[i];
        EXPECT_EQ(joint.geometry.source_joint_id,row.source_id);
        EXPECT_EQ(row.disposition,joint_source::Disposition::Required);
        for (unsigned k=0;k<2;++k) {
            EXPECT_EQ(joint.domain_nodes[k],Domain().domain().Find(row.nodes[k].source_id));
            EXPECT_EQ(Model().rigid_assembly().groups()[joint.body_groups[k]].source_id,row.nodes[k].body.source_id);
        }
        if (row.source_id>=2200526 && row.source_id<=2200529) ++rods;
        Same(joint.property.automatic_stiffness_scale,.01);
        Same(joint.property.critical_damping_ratio,.05);
    }
    EXPECT_EQ(rods,4u);
    RecordProperty("required_joints",value.model().joints().size());
    RecordProperty("joint_boundaries",source.data().boundaries);
    RecordProperty("joint_forecast",std::to_string(value.forecast().total_bytes));
}
} // namespace crash::cases::vehicle_startup::physical_model::supports_test
