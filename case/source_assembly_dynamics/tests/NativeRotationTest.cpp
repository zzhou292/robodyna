#include "case/source_assembly_dynamics/NativeRotation.h"
#include "case/source_assembly/tests/SourceAssemblyBindingTestSupport.h"
#include "lib_src/constraints/NodalRigidGroupMath.h"
#include "lib_src/elements/qeph/QephKinematics.h"
#include <gtest/gtest.h>
#include <cmath>
#include <cstring>
#include <limits>
namespace crash::cases::source_assembly_dynamics::test {
namespace value=tl::fea::rigid::detail;
namespace {
tl::math::Matrix3 RotateX(double a) {return {{1,0,0,0,std::cos(a),-std::sin(a),0,std::sin(a),std::cos(a)}};}
NativeRotationReference Reference() {
    NativeRotationReference r;r.source_parent=2214871;r.arity=4;r.frame=RotateX(0);
    for(auto& n:r.world_normals)n={0,0,1};return r;
}
constexpr tl::math::Vec3 Normals[4]{{0,0,1},{0,0,1},{0,0,1},{0,0,1}};
Config Configuration() {Config c;c.fixed_dt=1./67108864;c.deformation={.02,1,1,.2,.2,.5,1.5,.5,1.5,.5};return c;}
}
TEST(NativeRotation,FrameAndLocalNormalAreIndependentQualifiedGeometricMeasures) {
    const auto ref=Reference();NativeRotationMeasure m;
    ASSERT_TRUE(MeasureNativeRotation(ref,ref.frame,Normals,4,1,m));EXPECT_EQ(m.frame,0);EXPECT_EQ(m.nodal_normal,0);
    for(double angle:{1e-8,1.1,1.4}) {
        const auto frame=RotateX(angle);ASSERT_TRUE(MeasureNativeRotation(ref,frame,Normals,4,1.5,m));
        EXPECT_NEAR(m.frame,angle,1e-15);EXPECT_NEAR(m.nodal_normal,angle,1e-15);
        if(angle>1) {const auto old=m;const auto r=MeasureNativeRotation(ref,frame,Normals,4,1,m);
            EXPECT_FALSE(r);EXPECT_EQ(r.source_parent,2214871u);EXPECT_EQ(r.limit,1);EXPECT_GT(r.measured,1);
            EXPECT_EQ(std::memcmp(&old,&m,sizeof(m)),0);}
    }
    auto normal=Normals[0];normal={0,-std::sin(1.1),std::cos(1.1)};
    tl::math::Vec3 warped[]{Normals[0],Normals[1],Normals[2],normal};
    ASSERT_TRUE(MeasureNativeRotation(ref,ref.frame,warped,4,1.5,m));EXPECT_EQ(m.frame,0);EXPECT_NEAR(m.nodal_normal,1.1,1e-15);
    EXPECT_FALSE(MeasureNativeRotation(ref,ref.frame,warped,4,1,m));
}
TEST(NativeRotation,MalformedFramesLateNormalsLimitsAndAliasesPreserveOutput) {
    auto ref=Reference();NativeRotationMeasure m{.125,.25};const auto before=m;
    auto check=[&](const auto& r,const auto& f,const auto* normals,std::size_t arity,double limit) {
        EXPECT_FALSE(MeasureNativeRotation(r,f,normals,arity,limit,m));EXPECT_EQ(std::memcmp(&m,&before,sizeof(m)),0);
    };
    auto bad=ref.frame;bad.v[0]=-1;check(ref,bad,Normals,4,1);bad=ref.frame;bad.v[8]=1.01;check(ref,bad,Normals,4,1);
    for(double limit:{0.,-1.,1.50001,std::numeric_limits<double>::infinity()})check(ref,ref.frame,Normals,4,limit);
    check(ref,ref.frame,reinterpret_cast<const tl::math::Vec3*>(1),SIZE_MAX,1);
    tl::math::Vec3 late[]{Normals[0],Normals[1],Normals[2],Normals[3]};late[3].x=std::numeric_limits<double>::quiet_NaN();
    check(ref,ref.frame,late,4,1);late[3]={0,0,.5};check(ref,ref.frame,late,4,1);
    const auto saved=ref;auto& overlap=*reinterpret_cast<NativeRotationMeasure*>(&ref.frame);
    EXPECT_FALSE(MeasureNativeRotation(ref,ref.frame,Normals,4,1,overlap));EXPECT_EQ(std::memcmp(&ref,&saved,sizeof(ref)),0);
}
TEST(NativeRotation,ActualCompleteSourceHasExact915Parents1030NodesAnd76RigidMembers) {
    namespace source=source_assembly;const auto b=source::SourceAssemblyBindings::Prepare(source::test::Load(),source::test::Options());
    std::unique_ptr<const NativeRotationReferences> reference;const auto r=PrepareNativeRotation(b,1./67108864,reference);ASSERT_TRUE(r)<<r.message;
    ASSERT_NE(reference,nullptr);EXPECT_EQ(reference->parent_count(),915u);EXPECT_EQ(reference->node_count(),1030u);
    EXPECT_EQ(reference->source_instance(),b.source_instance_id());std::size_t members=0;
    for(std::size_t n=0;n<reference->node_count();++n)members+=reference->grouped(n);EXPECT_EQ(members,76u);EXPECT_FALSE(reference->grouped(459));
    const auto& s=b.shells();
    for(std::size_t e=0;e<reference->parent_count();++e) {
        const auto& p=reference->parent(e);EXPECT_EQ(p.source_parent,e<s.qeph_count()?s.qeph_source_id(e):s.t3_source_id(e-s.qeph_count()));
        EXPECT_EQ(p.arity,e<s.qeph_count()?4u:3u);EXPECT_TRUE(value::Orthonormal(p.frame));
        for(std::size_t n=0;n<p.arity;++n)EXPECT_NEAR(value::Dot(p.world_normals[n],p.world_normals[n]),1,1e-12);
    }
    // The actual warped shared-node normals remain source-specific and unequal.
    const auto a=reference->parent(392).world_normals[1],c=reference->parent(393).world_normals[0];
    EXPECT_LT(value::Dot(a,c),1-1e-8);const auto* old=reference.get();
    EXPECT_FALSE(PrepareNativeRotation(b,0,reference));EXPECT_EQ(reference.get(),old);
}
TEST(NativeRotation,OptInQualificationCapAndUnknownPoliciesFailClosedWithoutChangingLegacy) {
    auto c=Configuration();EXPECT_TRUE(ValidConfig(c));c.deformation.maximum_rotation=2;EXPECT_TRUE(ValidConfig(c));
    c.rotation_domain=RotationDomain::NativeShellGeometryV1;EXPECT_FALSE(ValidConfig(c));
    c.deformation.maximum_rotation=1.5;EXPECT_TRUE(ValidConfig(c));c.rotation_domain=static_cast<RotationDomain>(255);EXPECT_FALSE(ValidConfig(c));
}
}
