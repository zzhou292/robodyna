#include "AssemblyLongDoubleOracle.h"
#include <limits>

namespace rigid_assembly_test {
TEST(RigidAssemblyValues, OrderedPartExtrasAndPairMergeMatchIndependentFullTensor) {
  for(bool extras:{false,true})for(bool measurable_primary:{false,true}) {
    Fixture a,b;if(measurable_primary){a.ScaleMass(1e-20);b.ScaleMass(1e-20);}
    for(auto& p:b.part){p.position.x+=3;p.position.y-=.7;}
    b.primary.position.x+=3;b.primary.position.y-=.7;
    r::AssemblyRawBody av,bv,merged;
    ASSERT_EQ(r::PrepareAssemblyRawBody(a.Input(extras),av),S::Success);
    ASSERT_EQ(r::PrepareAssemblyRawBody(b.Input(false),bv),S::Success);
    ASSERT_EQ(r::MergeAssemblyRawBodies(av,bv,merged),S::Success);
    const auto ao=Oracle(a.Input(extras)),bo=Oracle(b.Input(false));
    Compare(av,ao);Compare(bv,bo);Compare(merged,Merge(ao,bo));
    EXPECT_EQ(merged.ledger.primaries,2u);EXPECT_EQ(merged.ledger.part_members,8u);
    EXPECT_DOUBLE_EQ(merged.ledger.primary_mass,a.primary.mass+b.primary.mass);
    EXPECT_DOUBLE_EQ(merged.ledger.primary_inertia,a.primary.inertia+b.primary.inertia);
  }
}
TEST(RigidAssemblyValues, ExtraBranchAndOriginalPrimariesCannotBeFlattenedAway) {
  Fixture f;f.ScaleMass(1e-20);r::AssemblyRawBody actual,wrong;
  ASSERT_EQ(r::PrepareAssemblyRawBody(f.Input(),actual),S::Success);
  std::array<r::AssemblyMassPoint,6> flattened;
  std::copy(f.part.begin(),f.part.end(),flattened.begin());std::copy(f.extra.begin(),f.extra.end(),flattened.begin()+4);
  auto in=f.Input(false);in.part=flattened.data();in.part_count=flattened.size();
  ASSERT_EQ(r::PrepareAssemblyRawBody(in,wrong),S::Success);
  double delta=0;for(unsigned i=0;i<9;++i)delta+=std::abs(actual.tensor.v[i]-wrong.tensor.v[i]);
  EXPECT_GT(delta,1e-22); // Source extra branch resets primary shift origin.
  Fixture b;b.ScaleMass(1e-20);r::AssemblyRawBody child,merged;
  ASSERT_EQ(r::PrepareAssemblyRawBody(b.Input(false),child),S::Success);
  ASSERT_EQ(r::MergeAssemblyRawBodies(actual,child,merged),S::Success);
  EXPECT_GT(merged.mass-(actual.mass+child.mass-b.primary.mass),.9*b.primary.mass);
}
TEST(RigidAssemblyValues, PrincipalCorrectionOccursAfterAllRawBodiesAreMerged) {
  Fixture a,b;
  for(unsigned i=0;i<4;++i){a.part[i]={{double(i)-1.5,0,0},1,1e-8};b.part[i]={{0,double(i)-1.5,0},1,1e-8};}
  a.primary.position=b.primary.position={0,0,0};
  r::AssemblyRawBody av,bv,merged; r::AssemblyFinalBody af,correct,early;
  ASSERT_EQ(r::PrepareAssemblyRawBody(a.Input(false),av),S::Success);
  ASSERT_EQ(r::PrepareAssemblyRawBody(b.Input(false),bv),S::Success);
  ASSERT_TRUE(r::FinalizeAssemblyRawBody(av,af));ASSERT_TRUE(af.regularization.principal_inertia_changed);
  ASSERT_EQ(r::MergeAssemblyRawBodies(bv,av,merged),S::Success);
  ASSERT_TRUE(r::FinalizeAssemblyRawBody(merged,correct));EXPECT_FALSE(correct.regularization.principal_inertia_changed);
  auto altered=av;altered.tensor=af.effective_tensor;
  ASSERT_EQ(r::MergeAssemblyRawBodies(bv,altered,merged),S::Success);
  ASSERT_TRUE(r::FinalizeAssemblyRawBody(merged,early));
  EXPECT_GT(std::abs(early.raw.tensor.v[0]-correct.raw.tensor.v[0]),.1);
}
TEST(RigidAssemblyValues, RejectionsPreserveOutputAndExactRetry) {
  Fixture f;r::AssemblyRawBody clean;
  ASSERT_EQ(r::PrepareAssemblyRawBody(f.Input(),clean),S::Success);auto out=clean;const auto bytes=Bytes(out);
  auto in=f.Input();in.part_count=SIZE_MAX;
  EXPECT_EQ(r::PrepareAssemblyRawBody(in,out),S::ResourceLimit);EXPECT_EQ(Bytes(out),bytes);
  in=f.Input();in.part=reinterpret_cast<const r::AssemblyMassPoint*>(&out);
  EXPECT_EQ(r::PrepareAssemblyRawBody(in,out),S::InvalidInput);EXPECT_EQ(Bytes(out),bytes);
  const auto saved=f.extra.back();f.extra.back().inertia=std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(r::PrepareAssemblyRawBody(f.Input(),out),S::InvalidInput);EXPECT_EQ(Bytes(out),bytes);
  f.extra.back()=saved;
  EXPECT_EQ(r::MergeAssemblyRawBodies(out,clean,out),S::InvalidInput);EXPECT_EQ(Bytes(out),bytes);
  auto malformed=clean;malformed.ledger.extra_mass*=2;
  EXPECT_EQ(r::MergeAssemblyRawBodies(clean,malformed,out),S::InvalidInput);EXPECT_EQ(Bytes(out),bytes);
  malformed=clean;malformed.center.x=std::numeric_limits<double>::max();
  EXPECT_EQ(r::MergeAssemblyRawBodies(clean,malformed,out),S::NonfiniteResult);EXPECT_EQ(Bytes(out),bytes);
  ASSERT_EQ(r::PrepareAssemblyRawBody(f.Input(),out),S::Success);Same(out,clean);
}
} // namespace rigid_assembly_test
