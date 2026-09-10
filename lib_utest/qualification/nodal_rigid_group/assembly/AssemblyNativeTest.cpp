#include "AssemblyNativeOracle.h"
#include "AssemblyLongDoubleOracle.h"

extern "C" void nodal_rigid_native_ispher2(const double*,double*,int*);
namespace rigid_assembly_test {
void Exact(const r::AssemblyRawBody& actual,const NativeValues& expected) {
  const auto av=Values(actual);
  for(std::size_t i=0;i<av.size();++i)EXPECT_DOUBLE_EQ(av[i],expected[i])<<"channel "<<i;
}
TEST(RigidAssemblyNative,OriginalPartExtraBranchesMatchPinnedMassCenterAndFullTensor) {
  for(bool extra:{false,true})for(double scale:{1.,1e-20})for(double shift:{0.,31.25}) {
    Fixture f;f.ScaleMass(scale);
    for(auto& p:f.part){p.position.x+=shift;p.position.z-=2*shift;}
    for(auto& p:f.extra){p.position.x+=shift;p.position.z-=2*shift;}
    f.primary.position.x+=shift;f.primary.position.z-=2*shift;
    r::AssemblyRawBody actual;
    ASSERT_EQ(r::PrepareAssemblyRawBody(f.Input(extra),actual),S::Success);
    Exact(actual,NativeRaw(f.Input(extra)));Compare(actual,Oracle(f.Input(extra)));
  }
}
TEST(RigidAssemblyNative,PairMergeConsumesNativeRawChildrenBeforePrincipalCorrection) {
  for(double scale:{1.,1e-20}) {
    Fixture a,b;a.ScaleMass(scale);b.ScaleMass(scale);
    for(auto& p:b.part){p.position.x+=2.5;p.position.y-=.75;}
    b.primary.position.x+=2.5;b.primary.position.y-=.75;
    r::AssemblyRawBody av,bv,merged;
    ASSERT_EQ(r::PrepareAssemblyRawBody(a.Input(),av),S::Success);
    ASSERT_EQ(r::PrepareAssemblyRawBody(b.Input(false),bv),S::Success);
    ASSERT_EQ(r::MergeAssemblyRawBodies(av,bv,merged),S::Success);
    Exact(merged,NativeMerge(NativeRaw(a.Input()),NativeRaw(b.Input(false))));
    r::AssemblyFinalBody final;ASSERT_TRUE(r::FinalizeAssemblyRawBody(merged,final));
    const double input[]{final.raw_principal_inertia.x,final.raw_principal_inertia.y,final.raw_principal_inertia.z};
    double native[3]{};int changed=0;nodal_rigid_native_ispher2(input,native,&changed);
    EXPECT_DOUBLE_EQ(final.principal.inertia.x,native[0]);
    EXPECT_DOUBLE_EQ(final.principal.inertia.y,native[1]);
    EXPECT_DOUBLE_EQ(final.principal.inertia.z,native[2]);
    EXPECT_EQ(final.regularization.principal_inertia_changed,changed!=0);
    EXPECT_EQ(final.raw.ledger.primaries,2u);
  }
}
TEST(RigidAssemblyNative,AdmasType5AddsOnlySuppliedTranslationalMass) {
  // This proves one producer's contribution. It does not supply the final IN of
  // auxiliary source nodes or qualify a zero-IN constrained owner.
  for(const auto base:std::array<std::array<double,3>,3>{{{0,0,.003},{2,.004,3},{1e-20,2e-20,7.380358}}}) {
    const auto native=NativePointMass(base[0],base[1],base[2]);
    EXPECT_DOUBLE_EQ(native[0],base[0]+base[2]);
    EXPECT_DOUBLE_EQ(native[1],base[1]);EXPECT_DOUBLE_EQ(native[2],base[2]);
  }
}
} // namespace rigid_assembly_test
