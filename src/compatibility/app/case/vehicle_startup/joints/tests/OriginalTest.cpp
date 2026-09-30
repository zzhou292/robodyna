#include "../VehicleJointModel.h"
#include <algorithm>
#include "case/vehicle_startup/physical_model/tests/Support.h"
namespace crash::cases::vehicle_startup::joints::test {
namespace native=tl::fea::type45;
namespace src=modelio::type45;
const Physical& PhysicalModel() {return physical_model::test::Actual();}
const Source& JointSource() {
    static const auto value=Source::Prepare(PhysicalModel().source_domain(),src::Policy::OriginalDirectSdiType45V1);
    return value;
}
TEST(VehicleJointModelOriginal,AllRetainedSourceRowsBindActualBodiesAndKeepBoundaryEvidence) {
    const auto value=VehicleJointModel::Prepare(PhysicalModel(),JointSource());
    EXPECT_TRUE(value.physical().SharesStorage(PhysicalModel()));
    EXPECT_TRUE(value.model().domain()->SharesStorage(PhysicalModel().source_domain().domain()));
    ASSERT_EQ(value.model().joints().size(),38u);
    ASSERT_EQ(value.source().data().rows.size(),44u);
    ASSERT_EQ(value.source_rows().size(),38u);
    const auto& groups=PhysicalModel().rigid_assembly().groups();
    std::size_t plain=0,part=0;
    std::vector<std::uint64_t> retained;
    for(std::size_t i=0;i<38;++i) {
        const auto& joint=value.model().joints()[i];
        const auto& row=JointSource().data().rows[value.source_rows()[i]];
        ASSERT_EQ(row.disposition,src::Disposition::Required);
        ASSERT_EQ(joint.geometry.source_joint_id,row.source_id);
        retained.push_back(row.source_id);
        const auto& property=JointSource().data().properties[row.property_index].value;
        EXPECT_EQ(joint.property.kind,property.kind);
        EXPECT_EQ(joint.property.automatic_stiffness_scale,.01);
        EXPECT_EQ(joint.property.critical_damping_ratio,.05);
        for(unsigned k=0;k<(joint.property.kind==native::Kind::Spherical?2u:3u);++k)
            ASSERT_EQ(joint.domain_nodes[k],row.nodes[k].domain_index);
        for(unsigned endpoint=0;endpoint<2;++endpoint) {
            ASSERT_LT(joint.body_groups[endpoint],groups.size());
            const auto& group=groups[joint.body_groups[endpoint]];
            EXPECT_EQ(group.source_id,row.nodes[endpoint].body.source_id);
            plain+=group.source_kind==tl::fea::RigidBindingSourceKind::NodalGroup;
            part+=group.source_kind==tl::fea::RigidBindingSourceKind::Part;
            EXPECT_EQ(joint.damping[endpoint].mass_kg,group.mass_kg);
            const auto j=group.principal.inertia;
            EXPECT_EQ(joint.damping[endpoint].mean_principal_inertia_kg_m2,(j.x+j.y+j.z)/3.);
        }
    }
    EXPECT_EQ(plain,48u);EXPECT_EQ(part,28u);
    for(const auto& row:JointSource().data().rows) if(row.disposition==src::Disposition::OmittedAssemblyBoundary)
        EXPECT_EQ(std::find(retained.begin(),retained.end(),row.source_id),retained.end());
    RecordProperty("required_joints",value.model().joints().size());
    RecordProperty("native_model_startup_bytes",std::to_string(value.model().startup_payload_bytes()));
    RecordProperty("complete_host_reservation",std::to_string(value.forecast().total_bytes));
}
TEST(VehicleJointModelOriginal,CompleteCapsAndNativeExactCapPreserveRetryAndBacking) {
    const auto f=VehicleJointModel::Preflight(PhysicalModel(),JointSource());
    Limits cap;cap.host_bytes=f.total_bytes-1;
    EXPECT_THROW(VehicleJointModel::Prepare(PhysicalModel(),JointSource(),cap),std::exception);
    cap.host_bytes=f.total_bytes;
    auto value=VehicleJointModel::Prepare(PhysicalModel(),JointSource(),cap);
    Limits exact;exact.model.max_host_bytes=value.model().startup_payload_bytes();
    const auto retry=VehicleJointModel::Prepare(PhysicalModel(),JointSource(),exact);
    EXPECT_EQ(retry.model().startup_payload_bytes(),exact.model.max_host_bytes);
    --exact.model.max_host_bytes;
    EXPECT_THROW(VehicleJointModel::Prepare(PhysicalModel(),JointSource(),exact),std::exception);
    const auto copied=value;
    EXPECT_TRUE(copied.model().SharesStorage(value.model()));
    EXPECT_EQ(copied.source_rows(),value.source_rows());
}
} // namespace crash::cases::vehicle_startup::joints::test
