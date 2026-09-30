#include "../ReferenceStorage.h"
#include "ReferenceComparison.h"
namespace crash::cases::vehicle_startup::test {
TEST(VehicleRigidReference, OwnershipRoleSurvivesStartupAndRejectionWithoutChangingReferenceMath) {
    tl::fea::qeph::ReferenceInput q;
    q.node_ids[0]=11;q.node_ids[1]=13;q.node_ids[2]=17;q.node_ids[3]=19;
    q.position[0]={0,0,0};q.position[1]={.04,0,0};q.position[2]={.04,.03,0};q.position[3]={0,.03,0};
    q.density=7800;q.young_modulus=210e9;q.poisson_ratio=.3;q.thickness=.004;
    ReferenceRow row;row.element_id=101;row.part_id=41;
    row.role=modelio::vehicle::SourceShellRole::OriginalRigidPart;row.rigid_root_index=7;
    detail::ReferenceStorage current,legacy;
    detail::Append(current,row,q);detail::Append(legacy,{},q);
    ASSERT_EQ(current.qeph.size(),1);Same(current.qeph[0],legacy.qeph[0]);
    EXPECT_EQ(current.rows[0].role,row.role);EXPECT_EQ(current.rows[0].rigid_root_index,7);
    EXPECT_EQ(legacy.rows[0].role,modelio::vehicle::SourceShellRole::ConstitutiveShell);
    EXPECT_EQ(legacy.rows[0].rigid_root_index,SIZE_MAX);
    const auto prior=current.qeph[0];q.position[3]=q.position[2];row.element_id=103;
    detail::Append(current,row,q);
    EXPECT_EQ(current.qeph.size(),1);Same(current.qeph[0],prior);
    EXPECT_NE(current.rows[1].status,ReferenceStatus::Success);
    EXPECT_EQ(current.rows[1].role,row.role);EXPECT_EQ(current.rows[1].rigid_root_index,7);
    EXPECT_EQ(current.rows[1].reference_index,SIZE_MAX);
}
}
