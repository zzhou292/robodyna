#include "PlasticityBindingFixture.h"
#include <limits>
#include <memory>
#include <utility>

namespace plasticity_binding_test {
namespace mat=tl::material;
void Linear(fe::ShellPlasticityMaterialInput& m) {
  m.curve_id=0; m.hardening=mat::ShellPlasticityHardeningKind::LinearLaw44;
  m.linear={5400,30000}; m.rate={true,8000,8,10000};
}
TEST(ShellPlasticityAnalyticBinding, MixedAndAllAnalyticCatalogsOwnScopeWithoutCurvePointers) {
  for(bool all:{false,true}) {
    Fixture f; Linear(f.materials[1]); if(all) Linear(f.materials[0]);
    auto input=f.catalog(); input.curve_count=all?0:1; if(all) input.curves=nullptr;
    fe::ShellBatchBinding native; ASSERT_EQ(native.Initialize(f.collection()).status,fe::ShellBindingStatus::Success);
    Binding a; ASSERT_EQ(a.Initialize(native,input).status,Status::Success);
    EXPECT_EQ(a.curve_count(),all?0u:1u); EXPECT_EQ(a.curve_point_count(),all?0u:3u);
    Binding b(a),c(std::move(b)); EXPECT_TRUE(a.SameScope(c));
    fe::sections::PointParameters q,t;
    ASSERT_TRUE(c.Parameters(fe::ShellBindingFamily::Qeph,0,&q));
    ASSERT_TRUE(c.Parameters(fe::ShellBindingFamily::T3,0,&t));
    EXPECT_EQ(t.curve.plastic_strain,nullptr); EXPECT_EQ(t.curve.yield_stress_pa,nullptr); EXPECT_EQ(t.curve.count,0u);
    EXPECT_EQ(t.linear.initial_yield_pa,5400); EXPECT_GT(t.plastic_hardening_pa,30000);
    if(all) EXPECT_EQ(q.curve.plastic_strain,nullptr); else EXPECT_EQ(q.curve.yield_stress_pa[0],2700);
    f.materials[1].linear.initial_yield_pa=1; f.yq[0]=-1;
    EXPECT_EQ(t.linear.initial_yield_pa,5400); if(!all) EXPECT_EQ(q.curve.yield_stress_pa[0],2700);
    f.materials[1].linear.initial_yield_pa=5401; f.yq[0]=2700;
    Binding changed; ASSERT_EQ(changed.Initialize(native,input).status,Status::Success);
    EXPECT_FALSE(c.SameScope(changed)); // Source material change in the OTHER family's scope also counts.
    f.materials[1].linear.initial_yield_pa=5400; f.materials[1].linear.tangent_modulus_pa=30001;
    Binding tangent; ASSERT_EQ(tangent.Initialize(native,input).status,Status::Success); EXPECT_FALSE(c.SameScope(tangent));
  }
}
TEST(ShellPlasticityAnalyticBinding, LateInvalidMaterialAndAmbiguousCurveModesPublishNothing) {
  for(unsigned fault=0;fault<7;++fault) {
    Fixture f; Linear(f.materials[1]); auto input=f.catalog(); input.curve_count=1;
    fe::ShellBatchBinding native; ASSERT_EQ(native.Initialize(f.collection()).status,fe::ShellBindingStatus::Success);
    switch(fault) {
      case 0:f.materials[1].linear.tangent_modulus_pa=3e6;break;
      case 1:f.materials[1].linear.initial_yield_pa=std::numeric_limits<double>::quiet_NaN();break;
      case 2:f.materials[1].curve_id=47;break;
      case 3:f.materials[1].rate={};break;
      case 4:f.materials[1].hardening=static_cast<mat::ShellPlasticityHardeningKind>(99);break;
      case 5:f.materials[0].linear.initial_yield_pa=1;break;
      case 6:input.curve_count=0;input.curves=nullptr;break; // Table parent still requires its real curve.
    }
    Binding a; const auto old=Bytes(a);
    EXPECT_EQ(a.Initialize(native,input).status,Status::InvalidMaterial); EXPECT_EQ(Bytes(a),old);
    Fixture clean; Linear(clean.materials[1]); auto retry=clean.catalog(); retry.curve_count=1;
    EXPECT_EQ(a.Initialize(native,retry).status,Status::Success);
  }
  Fixture f; Linear(f.materials[1]); fe::ShellBatchBinding native;
  ASSERT_EQ(native.Initialize(f.collection()).status,fe::ShellBindingStatus::Success);
  Binding a; const auto old=Bytes(a);
  EXPECT_EQ(a.Initialize(native,f.catalog()).status,Status::UnreferencedDeclaration); EXPECT_EQ(Bytes(a),old);
}
} // namespace plasticity_binding_test
