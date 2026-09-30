#include "Truth.h"
#include "lib_utils/BoundedArena.h"
namespace wall_observer_test {
TEST(MappedWallObserversHost, FixedTreeEnclosesBinary128TruthAcrossGrainBoundaries) {
  for(unsigned count:{1u,127u,128u,129u,263u,33001u}) {
    SCOPED_TRACE(count);Fixture f(count);auto seed=Fixture::Identity();
    ASSERT_TRUE(m::ApplyObservers(f.Reduce(),count,seed));
    const auto actual=f.Staged();ASSERT_EQ(f.storage.control.status,Code::Ok);
    CheckTruth(f.nodes.data(),count,actual);SameObserved(actual,f.Staged());
    EXPECT_EQ(actual.owner_id,7u);EXPECT_EQ(actual.base_epoch,2u);EXPECT_EQ(actual.attempt,3u);
  }
}
TEST(MappedWallObserversHost, SignedCancellationSubnormalsAndStrictPenetration) {
  Fixture f(3);
  const double moments[]={0x1p54,1,-0x1p54};
  for(unsigned i=0;i<3;++i)f.nodes[i].wall_moment.x=moments[i];
  const auto serial=f.Serial(),parallel=f.Staged();
  EXPECT_NE(Bits(serial.wall_moment.x),Bits(parallel.wall_moment.x));
  CheckTruth(f.nodes.data(),3,parallel);
  const double tiny=std::numeric_limits<double>::denorm_min();
  for(auto& n:f.nodes) {n.force={tiny,tiny,tiny,0};n.potential={tiny,tiny,tiny,0};
    n.wall_reaction={tiny,-0.,0};n.wall_moment={-tiny,tiny,-0.};n.surface_power=-tiny;n.stiffness={};}
  auto zero=Fixture::Identity();zero.maximum_penetration=-0.;
  const auto a=f.Staged(zero),b=f.Serial(zero);CheckTruth(f.nodes.data(),3,a);
  EXPECT_EQ(Bits(a.maximum_penetration),Bits(-0.));SameObserved(a,b);
  for(auto& n:f.nodes) {n.force={};n.potential={};n.wall_reaction={-0.,0.,-0.};n.wall_moment={};n.surface_power=-0.;}
  SameObserved(f.Staged(),f.Serial());
}
void FallbackMatches(Fixture& f) {
  auto staged=Fixture::Identity();ASSERT_FALSE(m::ApplyObservers(f.Reduce(),f.nodes.size(),staged));
  SameObserved(staged,Fixture::Identity());
  const auto serial=f.Serial();const auto control=f.storage.control;
  const auto parallel=f.Staged();SameObserved(parallel,serial);
  EXPECT_EQ(f.storage.control.status,control.status);EXPECT_EQ(f.storage.control.node,control.node);
  EXPECT_EQ(f.storage.control.parent,control.parent);
}
TEST(MappedWallObserversHost, ExtremeFinitePrefixesAndMalformedIntervalsRetainSerialPartialResult) {
  Fixture f(263);const auto original=f.nodes;
  // Successful legacy prefix, outside the sufficient fast-path proof.
  f.nodes[0].wall_moment.x=.375*DBL_MAX;f.nodes[1].wall_moment.x=.375*DBL_MAX;
  f.nodes[2].wall_moment.x=-.375*DBL_MAX;FallbackMatches(f);EXPECT_EQ(f.storage.control.status,Code::Ok);
  f.nodes=original;f.nodes[1].wall_moment.x=DBL_MAX;f.nodes[2].wall_moment.x=DBL_MAX;
  f.nodes[3].wall_moment.x=-DBL_MAX;FallbackMatches(f);
  EXPECT_EQ(f.storage.control.status,Code::NonFiniteArithmetic);EXPECT_EQ(f.storage.control.node,f.nodes[2].node);
  // Directed endpoint overflow precedes later bad nominal/signed values.
  f.nodes=original;f.nodes[1].force={0,0,DBL_MAX,0};f.nodes[2].force={0,0,DBL_MAX,0};
  f.nodes.back().surface_power=INFINITY;FallbackMatches(f);EXPECT_EQ(f.storage.control.node,f.nodes[1].node);
  // Negative leaf intervals can be hidden by a different reduction grouping;
  // those leaves always replay the source-order domain checks.
  f.nodes=original;f.nodes[0].potential={-1,-1,0,0};FallbackMatches(f);
  EXPECT_EQ(f.storage.control.node,f.nodes.front().node);
  f.nodes=original;f.nodes[170].potential.upper=std::numeric_limits<double>::quiet_NaN();
  f.nodes[200].surface_power=INFINITY;FallbackMatches(f);EXPECT_EQ(f.storage.control.node,f.nodes[170].node);
  f.nodes=original;ASSERT_EQ(f.Staged().node_count,263u);EXPECT_EQ(f.storage.control.status,Code::Ok);
  CheckTruth(f.nodes.data(),263,f.storage.result.diagnostics);
}
TEST(MappedWallObserversHost, CompleteTypedScratchForecastExactCapAndLateRetry) {
  m::Layout layout;ASSERT_TRUE(m::MakeLayout(7,263,2,SIZE_MAX,layout));
  EXPECT_EQ(layout.observer.count,3u);EXPECT_EQ(layout.observer.bytes,3*sizeof(m::ObserverSummary));
  const auto old=layout;ASSERT_FALSE(m::MakeLayout(7,263,2,layout.bytes-1,layout));
  EXPECT_EQ(layout.bytes,old.bytes);EXPECT_EQ(layout.observer.offset,old.observer.offset);
  ASSERT_TRUE(m::MakeLayout(7,263,2,layout.bytes,layout));
  std::vector<std::max_align_t> arena((layout.bytes+sizeof(std::max_align_t)-1)/sizeof(std::max_align_t));
  auto side=m::Bind(arena.data(),layout);
  EXPECT_EQ(reinterpret_cast<unsigned char*>(side.observer)-reinterpret_cast<unsigned char*>(arena.data()),
      static_cast<std::ptrdiff_t>(layout.observer.offset));
  EXPECT_FALSE(m::MakeLayout(7,SIZE_MAX,0,SIZE_MAX,layout));EXPECT_EQ(layout.bytes,old.bytes);
  EXPECT_EQ(m::ObserverBlocks(0),0u);EXPECT_EQ(m::ObserverBlocks(SIZE_MAX),0u);
  ASSERT_TRUE(m::MakeLayout(c::MaxVehicleNodalWallDeviceParents,c::MaxVehicleNodalWallDeviceNodes,1024,SIZE_MAX,layout));
  EXPECT_EQ(layout.observer.count,256u);EXPECT_EQ(layout.observer.bytes,36864u);
  EXPECT_TRUE(m::FiniteObserverPrefixes(524288,1));EXPECT_FALSE(m::FiniteObserverPrefixes(524289,1));
}
} // namespace wall_observer_test
