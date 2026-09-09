#include "lib_utest/q4_rectangular_integration_fixture.h"
#include <gtest/gtest.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <limits>
#include <vector>

namespace {
namespace sc=tlfea::contact;
namespace rect=sc::q4_rectangular;
namespace test=q4_rectangular_test;
using test::Fixture;
using test::Limits;
struct Scratch {
  std::vector<sc::Q4RectangularCell> leaves;
  std::vector<std::uint32_t> heap;
  explicit Scratch(unsigned size=sc::MaxQ4IntegrationLeaves):leaves(size),heap(size) {}
  sc::Q4RectangularScratch View() { return {leaves.data(),heap.data(),unsigned(leaves.size()),unsigned(heap.size())}; }
};
void Encloses(const sc::Q4CertifiedIntegral& value,long double exact,double limit) {
  EXPECT_LE(static_cast<long double>(value.lower),exact); EXPECT_GE(static_cast<long double>(value.upper),exact);
  EXPECT_LE(std::abs(static_cast<long double>(value.value)-exact),static_cast<long double>(value.error));
  EXPECT_LE(value.error,limit);
}
void Check(const sc::Q4RectangularResult& output,const test::Oracle& oracle,const sc::Q4IntegrationLimits& limits) {
  const auto& r=output.integration; ASSERT_TRUE(r.valid); long double sum=0;
  for (unsigned n=0;n<4;++n) {
    Encloses(r.force[n],oracle.force[n],limits.force_error); sum+=oracle.force[n];
    EXPECT_EQ(r.nodal.forces[n].x,-r.force[n].value);
    EXPECT_EQ(r.nodal.forces[n].y,0); EXPECT_EQ(r.nodal.forces[n].z,0);
    EXPECT_EQ(r.nodal.couples[n].x,0); EXPECT_EQ(r.nodal.couples[n].y,0); EXPECT_EQ(r.nodal.couples[n].z,0);
  }
  Encloses(r.resultant,sum,limits.force_error); Encloses(r.potential,oracle.potential,limits.energy_error);
  EXPECT_EQ(r.feature_id,73u); EXPECT_EQ(r.parent_element_id,42u); EXPECT_EQ(r.base_epoch,9u); EXPECT_EQ(r.attempt,7u);
  EXPECT_EQ(r.visited,2*r.leaf_count-1); EXPECT_LE(r.leaf_count,limits.max_leaves); EXPECT_LE(r.visited,limits.max_visited);
  EXPECT_EQ(r.deepest_leaf,std::max(output.deepest_u,output.deepest_v));
  EXPECT_LE(output.deepest_u,limits.max_depth); EXPECT_LE(output.deepest_v,limits.max_depth);
}
void Partition(const Scratch& scratch,const sc::Q4RectangularResult& result) {
  // Independent integer grid coverage: all coordinates on2^16, areas on2^32.
  // Positive rectangles, pairwise disjoint interiors and totalarea2^32 certify
  // exact coverage, independent of floating area/restriction calculations.
  struct Box { std::uint64_t u0,u1,v0,v1; };
  std::vector<Box> boxes; std::uint64_t area=0;
  for (unsigned i=0;i<result.integration.leaf_count;++i) {
    const auto& c=scratch.leaves[i]; ASSERT_LE(c.u_depth,16u); ASSERT_LE(c.v_depth,16u);
    const std::uint64_t du=1ULL<<(16-c.u_depth),dv=1ULL<<(16-c.v_depth);
    const Box b{c.bounds.column*du,(c.bounds.column+1)*du,c.bounds.row*dv,(c.bounds.row+1)*dv};
    ASSERT_LT(b.u0,b.u1); ASSERT_LT(b.v0,b.v1); ASSERT_LE(b.u1,65536u); ASSERT_LE(b.v1,65536u);
    for (const auto& a:boxes) EXPECT_TRUE(b.u0>=a.u1 || a.u0>=b.u1 || b.v0>=a.v1 || a.v0>=b.v1);
    boxes.push_back(b); area+=(b.u1-b.u0)*(b.v1-b.v0);
  }
  EXPECT_EQ(area,1ULL<<32);
}
void Overlap(const sc::Q4CertifiedIntegral& a,const sc::Q4CertifiedIntegral& b) {
  EXPECT_LE(a.lower,b.upper); EXPECT_LE(b.lower,a.upper);
  EXPECT_LE(std::abs(static_cast<long double>(a.value)-b.value),static_cast<long double>(a.error)+b.error);
}

TEST(Q4RectangularBounds, DyadicCornersAndAreaAtBothDepthLimitsAreExact) {
  for (const unsigned du:{0u,1u,7u,16u}) for (const unsigned dv:{0u,3u,16u}) {
    sc::Q4RectangularCell cell; cell.u_depth=du; cell.v_depth=dv; cell.bounds.depth=std::max(du,dv);
    cell.bounds.column=(1u<<du)-1; cell.bounds.row=(1u<<dv)/2;
    double shape[4][4]; ASSERT_TRUE(rect::Corners(cell,shape));
    const unsigned us[4]={1,0,0,1},vs[4]={1,1,0,0};
    for (unsigned corner=0;corner<4;++corner) {
      const long double s=std::ldexp(static_cast<long double>(cell.bounds.column+us[corner]),-int(du));
      const long double t=std::ldexp(static_cast<long double>(cell.bounds.row+vs[corner]),-int(dv));
      const long double expected[4]={s*t,(1-s)*t,(1-s)*(1-t),s*(1-t)};
      for (unsigned n=0;n<4;++n) EXPECT_EQ(static_cast<long double>(shape[corner][n]),expected[n]);
    }
    sc::Q4IntegralInterval area; ASSERT_TRUE(rect::Area(.1,cell,&area));
    const long double exact=std::ldexp(static_cast<long double>(.1),-int(du+dv));
    EXPECT_LE(static_cast<long double>(area.lower),exact); EXPECT_GE(static_cast<long double>(area.upper),exact);
  }
}

TEST(Q4RectangularIntegration, UniformBilinearInactiveAndTouchReuseTheOriginalScalarOperation) {
  Scratch scratch; Fixture fixture; auto limits=Limits(2); sc::Q4RectangularResult result;
  std::vector<sc::Q4IntegrationCell> scalar_leaves(sc::MaxQ4IntegrationLeaves);
  std::vector<std::uint32_t> scalar_heap(sc::MaxQ4IntegrationLeaves);
  for (unsigned kind=0;kind<4;++kind) {
    if (kind==0) fixture.Gaps(1,1,1,1);
    if (kind==1) fixture.Gaps(1.875,1.5,1,1.25);
    if (kind==2) fixture.Gaps(-1,-1,-1,-1);
    if (kind==3) fixture.Gaps(0,0,0,0);
    ASSERT_EQ(sc::IntegrateQ4NormalContactRectangular(fixture.Input(),limits,scratch.View(),&result).status,sc::Q4IntegrationStatus::Ok);
    sc::Q4IntegrationResult scalar;
    ASSERT_EQ(sc::IntegrateQ4NormalContact(fixture.Input(),limits,{scalar_leaves.data(),scalar_heap.data(),4096,4096},&scalar).status,
              sc::Q4IntegrationStatus::Ok);
    Check(result,kind==0?q4_contact_test::Uniform(1):kind==1?q4_contact_test::Bilinear(1,.25,.5,.125):test::Oracle{},limits);
    EXPECT_EQ(result.integration.leaf_count,1u); EXPECT_EQ(result.integration.resultant.value,scalar.resultant.value);
    EXPECT_EQ(result.integration.potential.value,scalar.potential.value);
    for (unsigned n=0;n<4;++n) {
      EXPECT_EQ(result.integration.force[n].value,scalar.force[n].value);
      EXPECT_EQ(result.integration.force[n].lower,scalar.force[n].lower); EXPECT_EQ(result.integration.force[n].upper,scalar.force[n].upper);
    }
  }
}

TEST(Q4RectangularIntegration, CutCornerSaddleAndNonzeroWidthVariationHaveIndependentIntegrals) {
  Scratch scratch; Fixture fixture; sc::Q4IntegrationLimits limits;
  for (unsigned kind=0;kind<5;++kind) {
    SCOPED_TRACE(kind); test::Configure(kind,fixture,limits); sc::Q4RectangularResult result;
    ASSERT_EQ(sc::IntegrateQ4NormalContactRectangular(fixture.Input(),limits,scratch.View(),&result).status,sc::Q4IntegrationStatus::Ok);
    Check(result,test::Expected(kind),limits); Partition(scratch,result);
    if (kind==0 || kind==3) { EXPECT_GT(result.deepest_u,0u); EXPECT_EQ(result.deepest_v,0u); }
    if (kind==2) { EXPECT_EQ(result.deepest_u,1u); EXPECT_EQ(result.deepest_v,1u); EXPECT_EQ(result.integration.leaf_count,4u); }
    if (kind==3) EXPECT_NE(fixture.position[fixture.parent.nodes[0]],fixture.position[fixture.parent.nodes[3]]);
  }
}

TEST(Q4RectangularIntegration, AxisInterchangePermutesForcesMassMapAndDirectionalDepths) {
  Scratch scratch; Fixture fixture; fixture.Gaps(.625,-.375,-.375,.625); const auto limits=Limits();
  sc::Q4RectangularResult u,v;
  ASSERT_EQ(sc::IntegrateQ4NormalContactRectangular(fixture.Input(),limits,scratch.View(),&u).status,sc::Q4IntegrationStatus::Ok);
  // Natural U/V interchange swaps physical Y/Z and localnodes1/3 together;
  // physical node masses/masks/velocities remain attached to their real IDs.
  std::swap(fixture.parent.nodes[1],fixture.parent.nodes[3]);
  for (unsigned n=0;n<4;++n) std::swap(fixture.position[n+4],fixture.position[n+8]);
  ASSERT_EQ(sc::IntegrateQ4NormalContactRectangular(fixture.Input(),limits,scratch.View(),&v).status,sc::Q4IntegrationStatus::Ok);
  const unsigned map[4]={0,3,2,1};
  for (unsigned n=0;n<4;++n) { Overlap(v.integration.force[n],u.integration.force[map[n]]); EXPECT_EQ(v.integration.nodal.nodes[n],u.integration.nodal.nodes[map[n]]); }
  Overlap(v.integration.potential,u.integration.potential); Overlap(v.integration.resultant,u.integration.resultant);
  EXPECT_EQ(v.deepest_u,u.deepest_v); EXPECT_EQ(v.deepest_v,u.deepest_u); Partition(scratch,v);
}

TEST(Q4RectangularIntegration, ThinStripsAcrossCoarseDyadicBoundariesAndObliqueCutRemainCertified) {
  Scratch scratch; Fixture fixture;
  for (const double a:{.375+std::ldexp(1.,-12),1-std::ldexp(1.,-10)}) {
    fixture.Gaps(1-a,-a,-a,1-a); auto limits=Limits(); sc::Q4RectangularResult result;
    ASSERT_EQ(sc::IntegrateQ4NormalContactRectangular(fixture.Input(),limits,scratch.View(),&result).status,sc::Q4IntegrationStatus::Ok);
    Check(result,test::ObliqueCut(a,0),limits); EXPECT_EQ(result.deepest_v,0u); Partition(scratch,result);
  }
  fixture.Gaps(.75,-.25,-.375,.625); sc::Q4RectangularResult result;
  ASSERT_EQ(sc::IntegrateQ4NormalContactRectangular(fixture.Input(),Limits(),scratch.View(),&result).status,sc::Q4IntegrationStatus::Ok);
  Check(result,test::ObliqueCut(.375,.125),Limits()); Partition(scratch,result);
}

TEST(Q4RectangularIntegration, NearOneDimensionalPartitionReducesLeavesAgainstIntactScalarOracle) {
  Fixture fixture; sc::Q4IntegrationLimits limits; test::Configure(3,fixture,limits); Scratch scratch;
  sc::Q4RectangularResult result; sc::Q4IntegrationResult scalar;
  std::vector<sc::Q4IntegrationCell> leaves(sc::MaxQ4IntegrationLeaves); std::vector<std::uint32_t> heap(sc::MaxQ4IntegrationLeaves);
  ASSERT_EQ(sc::IntegrateQ4NormalContactRectangular(fixture.Input(),limits,scratch.View(),&result).status,sc::Q4IntegrationStatus::Ok);
  ASSERT_EQ(sc::IntegrateQ4NormalContact(fixture.Input(),limits,{leaves.data(),heap.data(),4096,4096},&scalar).status,sc::Q4IntegrationStatus::Ok);
  Check(result,test::Expected(3),limits);
  for (unsigned n=0;n<4;++n) Overlap(result.integration.force[n],scalar.force[n]);
  Overlap(result.integration.potential,scalar.potential); Overlap(result.integration.resultant,scalar.resultant);
  EXPECT_LT(result.integration.leaf_count,scalar.leaf_count); // Algorithmic work count, not a timing claim.
}

TEST(Q4RectangularIntegration, NonbinaryAreaStiffnessAndCommonWorldOffsetRespectExactInputScale) {
  Fixture fixture; fixture.Gaps(.25,.25,.25,.25); Scratch scratch; auto input=fixture.Input();
  for (unsigned n=0;n<4;++n) fixture.position[n+4]*=.1;
  input.projected_area=.1; input.stiffness_per_area=16;
  auto oracle=q4_contact_test::Uniform(.25); const long double scale=static_cast<long double>(.1)*16;
  for (auto& force:oracle.force) force*=scale; oracle.potential*=scale;
  sc::Q4RectangularResult base,shifted;
  ASSERT_EQ(sc::IntegrateQ4NormalContactRectangular(input,Limits(),scratch.View(),&base).status,sc::Q4IntegrationStatus::Ok);
  Check(base,oracle,Limits());
  for (unsigned n=0;n<4;++n) fixture.position[n]+=8; input.wall_x=8;
  ASSERT_EQ(sc::IntegrateQ4NormalContactRectangular(input,Limits(),scratch.View(),&shifted).status,sc::Q4IntegrationStatus::Ok);
  EXPECT_EQ(base.integration.resultant.value,shifted.integration.resultant.value);
  EXPECT_EQ(base.integration.potential.value,shifted.integration.potential.value); Check(shifted,oracle,Limits());
}

TEST(Q4RectangularIntegration, SelectedSecondParentRetainsPhysicalIdsAndMappedMassSlots) {
  Fixture fixture; fixture.Gaps(.625,-.375,-.375,.625); Scratch scratch;
  const sc::SurfaceQ4 parents[2]={{{0,1,2,3},991,992,3,0},fixture.parent};
  auto input=fixture.Input(); input.surface.parents=parents; input.surface.parent_count=2; input.parent_index=1;
  sc::Q4RectangularResult result;
  ASSERT_EQ(sc::IntegrateQ4NormalContactRectangular(input,Limits(),scratch.View(),&result).status,sc::Q4IntegrationStatus::Ok);
  Check(result,q4_contact_test::Cut(.375),Limits());
  for (unsigned n=0;n<4;++n) EXPECT_EQ(result.integration.nodal.nodes[n],fixture.parent.nodes[n]);
  EXPECT_NE(result.integration.feature_id,parents[0].feature_id);
}

TEST(Q4RectangularIntegration, EveryHardLimitPreservesEveryOutputByteAndAllowsCleanRetry) {
  Fixture fixture; Scratch scratch; const double e=1./64; fixture.Gaps(e-2,e-1,e,e-1);
  for (unsigned kind=0;kind<4;++kind) {
    auto limits=Limits(e); if (kind==0) limits.max_leaves=1; if (kind==1) limits.max_visited=1;
    if (kind==2) limits.max_depth=0; if (kind==3) limits.max_depth=sc::MaxQ4IntegrationDepth+1;
    sc::Q4RectangularResult output; std::memset(&output,0xa5,sizeof(output));
    std::array<unsigned char,sizeof(output)> before; std::memcpy(before.data(),&output,sizeof(output));
    const sc::Q4IntegrationStatus expected[4]={sc::Q4IntegrationStatus::LeafLimit,sc::Q4IntegrationStatus::VisitLimit,
        sc::Q4IntegrationStatus::DepthLimit,sc::Q4IntegrationStatus::InvalidInput};
    EXPECT_EQ(sc::IntegrateQ4NormalContactRectangular(fixture.Input(),limits,scratch.View(),&output).status,expected[kind]);
    EXPECT_EQ(std::memcmp(before.data(),&output,sizeof(output)),0);
    Fixture clean; clean.Gaps(1,1,1,1);
    ASSERT_EQ(sc::IntegrateQ4NormalContactRectangular(clean.Input(),Limits(),scratch.View(),&output).status,sc::Q4IntegrationStatus::Ok);
    Check(output,q4_contact_test::Uniform(1),Limits());
  }
}

TEST(Q4RectangularIntegration, LateGaussMassFailureAndLostPositiveEnergyNeverPublish) {
  Scratch scratch; Fixture fixture; fixture.Gaps(1,1,1,1);
  for (unsigned kind=0;kind<2;++kind) {
    Fixture bad=fixture;
    if (kind==0) for (unsigned n=0;n<4;++n) if (bad.fixed[n]==6) bad.inverse[n]=16*std::numeric_limits<double>::denorm_min();
    if (kind==1) bad.Gaps(1e-200,1e-200,1e-200,1e-200);
    sc::NormalJacobian center; ASSERT_EQ(sc::BuildQ4NormalXJacobian(bad.Input().mass,bad.parent,0,0,7,&center),sc::Status::kOk);
    sc::Q4RectangularResult output; std::memset(&output,0xa5,sizeof(output));
    std::array<unsigned char,sizeof(output)> before; std::memcpy(before.data(),&output,sizeof(output));
    const auto report=sc::IntegrateQ4NormalContactRectangular(bad.Input(),Limits(),scratch.View(),&output);
    EXPECT_EQ(report.status,sc::Q4IntegrationStatus::NonFiniteArithmetic); EXPECT_EQ(std::memcmp(before.data(),&output,sizeof(output)),0);
    ASSERT_EQ(sc::IntegrateQ4NormalContactRectangular(fixture.Input(),Limits(),scratch.View(),&output).status,sc::Q4IntegrationStatus::Ok);
    Check(output,q4_contact_test::Uniform(1),Limits());
  }
}

TEST(Q4RectangularBounds, AxisTieDepthFallbackAndOrderingOverflowAreExplicit) {
  sc::Q4RectangularCell cell; auto limits=Limits(); bool u=false;
  const sc::Q4IntegralInterval saddle[4]={{1,1},{-1,-1},{1,1},{-1,-1}};
  ASSERT_TRUE(rect::SplitAxis(cell,saddle,limits,&u)); EXPECT_TRUE(u);
  cell.u_depth=limits.max_depth; cell.bounds.depth=cell.u_depth;
  ASSERT_TRUE(rect::SplitAxis(cell,saddle,limits,&u)); EXPECT_FALSE(u);
  cell.v_depth=limits.max_depth; EXPECT_FALSE(rect::SplitAxis(cell,saddle,limits,&u));
  EXPECT_TRUE(std::isinf(rect::DifferenceScore({-DBL_MAX,-DBL_MAX},{DBL_MAX,DBL_MAX})));
  cell={}; cell.bounds.column=1; double shape[4][4]; EXPECT_FALSE(rect::Corners(cell,shape));
}

TEST(Q4RectangularIntegration, InvalidInputAndScratchContractsPreserveCompleteCallerResult) {
  Scratch scratch; Fixture fixture; fixture.Gaps(1,1,1,1);
  for (unsigned kind=0;kind<9;++kind) {
    auto input=fixture.Input(); auto limits=Limits(); auto storage=scratch.View();
    if (kind==0) input.parent_index=1;
    if (kind==1) input.projected_area=0;
    if (kind==2) input.wall_x=std::numeric_limits<double>::quiet_NaN();
    if (kind==3) limits.max_leaves=sc::MaxQ4IntegrationLeaves+1;
    if (kind==4) limits.max_visited=sc::MaxQ4IntegrationVisits+1;
    if (kind==5) limits.energy_error=0;
    if (kind==6) storage.leaf_capacity=limits.max_leaves-1;
    if (kind==7) storage.heap_capacity=limits.max_leaves-1;
    if (kind==8) storage.leaves=nullptr;
    sc::Q4RectangularResult output; std::memset(&output,0xa5,sizeof(output));
    std::array<unsigned char,sizeof(output)> before; std::memcpy(before.data(),&output,sizeof(output));
    EXPECT_EQ(sc::IntegrateQ4NormalContactRectangular(input,limits,storage,&output).status,sc::Q4IntegrationStatus::InvalidInput)<<kind;
    EXPECT_EQ(std::memcmp(before.data(),&output,sizeof(output)),0)<<kind;
  }
}
} // namespace
