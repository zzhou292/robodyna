#include "Fixture.h"
#include "ContactResultComparison.h"
#include <cmath>
namespace crash::cases::source_assembly_dynamics {
Report SourceAssemblyDynamicsTestAccess::CheckRotationTrial(SourceAssemblyWallCase& c,RotationFault fault) {
    auto& s=*c.impl_;auto r=s.Prepare();if(!r)return s.Stop(r);
    r=s.Evaluate();if(!r)return s.Stop(r);
    if(fault==RotationFault::LastTriangleFrame) {
        const auto& source=s.rotation_reference->parent(s.quads()+s.triangles()-1);
        auto& frame=s.candidate().parents.t3.back().kinematics.frame;frame=source.frame;
        // A proper 1.1rad rotation of the final parent, introduced only in the
        // private copied observation packet. No fake candidate is committed.
        for(unsigned j=0;j<3;++j) {
            frame.v[3+j]=std::cos(1.1)*source.frame.v[3+j]-std::sin(1.1)*source.frame.v[6+j];
            frame.v[6+j]=std::sin(1.1)*source.frame.v[3+j]+std::cos(1.1)*source.frame.v[6+j];
        }
    } else {
        const auto n=fault==RotationFault::OrdinaryQuaternion?459u:s.bindings.rigid_groups()->members()[0].global_node;
        auto* q=s.candidate().fields.orientation.data()+4*n;q[0]=std::cos(.55);q[1]=std::sin(.55);q[2]=q[3]=0;
    }
    r=s.CheckRotations();const auto discarded=s.Stop(Failure(Status::EnvelopeFailure,"Test observation trial discarded"));
    if(discarded.status==Status::DeviceFailure)return discarded;return r;
}
namespace test {
using Access=SourceAssemblyDynamicsTestAccess;
TEST_F(SourceAssemblyDynamicsCheck,NativeRotationActual64ContactIntervalsKeepPhysicsAndAllocationsIdentical) {
    const auto b=source::SourceAssemblyBindings::Prepare(source::test::Load(),source::test::Options());
    const auto wall=PrepareWall(b);auto config=SmokeConfig();SourceAssemblyWallCase legacy,native;
    ASSERT_TRUE(legacy.Initialize(b,wall,config));config.rotation_domain=RotationDomain::NativeShellGeometryV1;
    const auto r=native.Initialize(b,wall,config);ASSERT_TRUE(r)<<r.message;
    EXPECT_EQ(native.allocations().device_bytes,legacy.allocations().device_bytes);
    EXPECT_EQ(native.allocations().device_allocations,legacy.allocations().device_allocations);
    const auto extra=sizeof(NativeRotationReferences)+915*sizeof(NativeRotationReference)+1030*sizeof(std::uint8_t);
    EXPECT_EQ(native.host_payload_bytes()-legacy.host_payload_bytes(),extra);
    EXPECT_EQ(native.diagnostics()->maximum_native_frame_rotation,0);EXPECT_EQ(native.diagnostics()->maximum_native_normal_rotation,0);
    EXPECT_EQ(native.diagnostics()->rotation_domain,RotationDomain::NativeShellGeometryV1);
    SourceAssemblyWallCase small;config.storage.max_host_bytes=native.host_payload_bytes()-1;
    EXPECT_EQ(small.Initialize(b,wall,config).status,Status::ResourceLimit);EXPECT_EQ(small.allocations().device_bytes,0u);
    const auto allocations=native.allocations();const auto host=native.host_payload_bytes();
    for(unsigned step=0;step<64;++step) {
        SCOPED_TRACE(step);ASSERT_TRUE(legacy.Step());const auto next=native.Step();ASSERT_TRUE(next)<<next.message;
        const auto& a=Access::Accepted(legacy);const auto& c=Access::Accepted(native);
        SameFields(a.fields,c.fields);SameParents(a.parents,c.parents);
        EXPECT_EQ(a.diagnostics.maximum_rotation,c.diagnostics.maximum_rotation);
        EXPECT_EQ(a.diagnostics.native_internal_work,c.diagnostics.native_internal_work);
        EXPECT_EQ(a.diagnostics.cumulative_plastic_work,c.diagnostics.cumulative_plastic_work);
        EXPECT_EQ(a.wall.diagnostics.resultant.value,c.wall.diagnostics.resultant.value);EXPECT_EQ(a.wall.wall_face,c.wall.wall_face);
        for(std::size_t i=0;i<a.wall.nodes.size();++i)EXPECT_EQ(ContactDifference(a.wall.nodes[i],c.wall.nodes[i]).field_bytes,0u);
        for(std::size_t i=0;i<a.wall.parents.size();++i)EXPECT_EQ(ContactDifference(a.wall.parents[i],c.wall.parents[i]).field_bytes,0u);
        EXPECT_LE(c.diagnostics.maximum_native_frame_rotation,1);EXPECT_LE(c.diagnostics.maximum_native_normal_rotation,1);
        EXPECT_LE(c.diagnostics.maximum_rigid_member_rotation,1);SameAllocations(native,allocations,host);
    }
    EXPECT_EQ(native.diagnostics()->first_contact_epoch,42u);EXPECT_GT(native.accepted_contact().diagnostics->resultant.value,0);
}
TEST_F(SourceAssemblyDynamicsCheck,NativeRotationProxyScopeLateFrameFailurePreservesAcceptedHistoryAndExactRetry) {
    const auto b=source::SourceAssemblyBindings::Prepare(source::test::Load(),source::test::Options());
    const auto wall=PrepareWall(b);auto config=SmokeConfig();SourceAssemblyWallCase legacy,native,reference;
    ASSERT_TRUE(legacy.Initialize(b,wall,config));config.rotation_domain=RotationDomain::NativeShellGeometryV1;
    ASSERT_TRUE(native.Initialize(b,wall,config));ASSERT_TRUE(reference.Initialize(b,wall,config));
    for(unsigned step=0;step<64;++step){ASSERT_TRUE(legacy.Step());ASSERT_TRUE(native.Step());ASSERT_TRUE(reference.Step());}
    const auto saved=Access::Accepted(native);const auto allocations=native.allocations();const auto host=native.host_payload_bytes();
    auto r=Access::CheckRotationTrial(legacy,Access::RotationFault::OrdinaryQuaternion);
    EXPECT_EQ(r.status,Status::EnvelopeFailure);EXPECT_STREQ(r.message,"Source total orientation envelope exceeded");
    r=Access::CheckRotationTrial(native,Access::RotationFault::OrdinaryQuaternion);EXPECT_TRUE(r)<<r.message;
    SameAccepted(saved,Access::Accepted(native));
    r=Access::CheckRotationTrial(native,Access::RotationFault::RigidQuaternion);
    EXPECT_EQ(r.status,Status::EnvelopeFailure);EXPECT_STREQ(r.message,"Source rigid-member total orientation envelope exceeded");
    SameAccepted(saved,Access::Accepted(native));
    r=Access::CheckRotationTrial(native,Access::RotationFault::LastTriangleFrame);
    EXPECT_EQ(r.status,Status::EnvelopeFailure);EXPECT_EQ(r.source_parent,b.shells().t3_source_id(b.shells().t3_count()-1));
    EXPECT_NEAR(r.measured,1.1,1e-14);EXPECT_EQ(r.limit,1);SameAccepted(saved,Access::Accepted(native));
    EXPECT_EQ(native.diagnostics()->maximum_native_frame_rotation,saved.diagnostics.maximum_native_frame_rotation);
    EXPECT_EQ(native.diagnostics()->maximum_native_normal_rotation,saved.diagnostics.maximum_native_normal_rotation);
    ASSERT_TRUE(native.Step());ASSERT_TRUE(reference.Step());
    SameFields(Access::Accepted(reference).fields,Access::Accepted(native).fields);
    SameParents(Access::Accepted(reference).parents,Access::Accepted(native).parents);
    EXPECT_EQ(native.diagnostics()->maximum_native_frame_rotation,reference.diagnostics()->maximum_native_frame_rotation);
    EXPECT_EQ(native.diagnostics()->maximum_native_normal_rotation,reference.diagnostics()->maximum_native_normal_rotation);
    SameAllocations(native,allocations,host);
}
}
}
