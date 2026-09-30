// SPDX-License-Identifier: MIT
#include "native/Packet.h"
#include "../tied_shell_search/NativeOracle.h"
#include <gtest/gtest.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <set>
#include <vector>

namespace {
namespace ts=tl::constraints::tied_shell;
struct Fixture {
  std::vector<ts::Vec3> x{{0,0,0},{10,0,0},{10,10,0},{0,10,0},
                        {0,0,0},{10,0,0},{10,10,0},{0,10,0}};
  std::vector<std::array<int,4>> masters{{1,2,3,4},{5,6,7,8},{1,2,3,3}};
  std::vector<int> msr{8,7,6,5,4,3,2,1},nsv;
  std::vector<double> thickness{1,1,1};
  Fixture() {
    // More than MVSIZ129 pairs exercises both full and final native batches.
    for(int i=0;i<400;++i) {
      x.push_back({-1.+(i%23)*.55,-1.+(i/23)*.7,(i%9-4)*.15});
      nsv.push_back(static_cast<int>(x.size()));
    }
    nsv.push_back(1); // Native physical-own-node exclusion, not coincident x.
    x.push_back({100,100,100}); nsv.push_back(static_cast<int>(x.size()));
  }
};
struct Output {
  std::vector<int> selected,pairs;
  std::vector<double> st,dist;
  std::array<double,6> bounds{};
  std::array<int,3> cells{};
  int count=-1,status=-1;
  explicit Output(int ns,int capacity):selected(ns,-99),pairs(2*capacity,-99),
      st(2*ns,-99),dist(ns,-99) {}
};
void Invoke(const Fixture& f,int capacity,Output& out) {
  static_assert(sizeof(ts::Vec3)==3*sizeof(double));
  native_tied_bucket(f.x.size(),f.masters.size(),f.nsv.size(),f.msr.size(),capacity,
      &f.x[0].x,f.masters[0].data(),f.nsv.data(),f.msr.data(),
      f.thickness.data(),f.thickness.data(),out.selected.data(),out.st.data(),out.dist.data(),
      &out.count,out.pairs.data(),out.bounds.data(),out.cells.data(),&out.status);
}
extern "C" void tied_native_bounds(const double*,const double*,const double*,double*);
TEST(TiedSearchBucketNative, CompleteBucketTraversalMatchesIndependentExhaustiveNativeSmallFixture) {
  const Fixture f;
  Output actual(f.nsv.size(),2048);
  Invoke(f,2048,actual);
  ASSERT_EQ(actual.status,0);
  ASSERT_GT(actual.count,129);
  std::set<std::pair<int,int>> enumerated;
  for(int p=0;p<actual.count;++p)
    ASSERT_TRUE(enumerated.emplace(actual.pairs[2*p],actual.pairs[2*p+1]).second);
  std::set<std::pair<int,int>> expected_pairs;
  const double secondary=0;
  for(std::size_t s=0;s<f.nsv.size();++s) {
    tied_search_test::NativeChoice choice;
    for(std::size_t m=0;m<f.masters.size();++m) {
      const auto& indices=f.masters[m];
      if(std::find(indices.begin(),indices.end(),f.nsv[s])!=indices.end()) continue;
      ts::WorkingSearchInput in;
      in.working_length_to_m=.001;
      in.master_thickness=f.thickness[m];
      in.topology=indices[2]==indices[3]?ts::MasterTopology::TriangleRepeatedThird:ts::MasterTopology::Quad;
      double coords[15];
      for(unsigned j=0;j<5;++j) {
        const auto v=f.x[j==4?f.nsv[s]-1:indices[j]-1];
        coords[3*j]=v.x;coords[3*j+1]=v.y;coords[3*j+2]=v.z;
        if(j==4)in.geometry.secondary_position=v;else in.geometry.master_position[j]=v;
      }
      double bounds[8];
      tied_native_bounds(coords,&f.thickness[m],&secondary,bounds);
      if(bounds[7]==0)continue;
      expected_pairs.emplace(m+1,s+1);
      (void)tied_search_test::Native(in,m+1,choice);
    }
    EXPECT_EQ(actual.selected[s],choice.selected)<<s;
    EXPECT_DOUBLE_EQ(actual.dist[s],choice.distance)<<s;
    EXPECT_DOUBLE_EQ(actual.st[2*s],choice.st[0])<<s;
    EXPECT_DOUBLE_EQ(actual.st[2*s+1],choice.st[1])<<s;
  }
  EXPECT_EQ(enumerated,expected_pairs);
}
TEST(TiedSearchBucketNative, CountFailurePreservesEveryOutputThenExactCapacityRetry) {
  const Fixture f;
  Output measured(f.nsv.size(),2048);Invoke(f,2048,measured);ASSERT_EQ(measured.status,0);
  Output next(f.nsv.size(),measured.count);
  const auto prior=next;
  Invoke(f,measured.count-1,next);
  EXPECT_EQ(next.status,2);
  EXPECT_EQ(next.selected,prior.selected);EXPECT_EQ(next.st,prior.st);EXPECT_EQ(next.dist,prior.dist);
  EXPECT_EQ(next.count,prior.count);EXPECT_EQ(next.pairs,prior.pairs);
  EXPECT_EQ(next.bounds,prior.bounds);EXPECT_EQ(next.cells,prior.cells);
  Invoke(f,measured.count,next);ASSERT_EQ(next.status,0);
  EXPECT_EQ(next.selected,measured.selected);EXPECT_EQ(next.st,measured.st);EXPECT_EQ(next.dist,measured.dist);
  EXPECT_EQ(next.count,measured.count);
}
TEST(TiedSearchBucketNative, InvalidLateNodeAndZeroNativeCellScaleRejectBeforePublication) {
  Fixture f;Output next(f.nsv.size(),2048);const auto prior=next;
  f.x.back().z=std::numeric_limits<double>::infinity();Invoke(f,2048,next);
  EXPECT_EQ(next.status,1);EXPECT_EQ(next.selected,prior.selected);EXPECT_EQ(next.count,prior.count);
  f.x.back().z=100;
  for(int i=0;i<8;++i)f.x[i]={0,0,0};
  Invoke(f,2048,next);EXPECT_EQ(next.status,1);EXPECT_EQ(next.selected,prior.selected);
  EXPECT_EQ(next.st,prior.st);EXPECT_EQ(next.dist,prior.dist);EXPECT_EQ(next.bounds,prior.bounds);
}
TEST(TiedSearchBucketNative, DomainRetainsNativeFirstDiagonalCellScaleAndLargerSegmentInflation) {
  Fixture f;
  f.x={{0,0,0},{100,0,0},{1,1,0},{0,100,0},{.2,.2,.1}};
  f.masters={{1,2,3,4}};f.msr={4,2,3,1};f.nsv={5};f.thickness={1};
  Output result(1,1);Invoke(f,1,result);ASSERT_EQ(result.status,0);
  const double margin=.05*std::sqrt(20000.);
  EXPECT_DOUBLE_EQ(result.bounds[0],-margin);
  EXPECT_DOUBLE_EQ(result.bounds[3],100+margin);
  // Native DD's second update reuses diagonal13. Replacing it with the
  // segment's larger diagonal changes these counts from20/20/2 to1/1/1.
  EXPECT_EQ(result.cells,(std::array<int,3>{20,20,2}));
  EXPECT_EQ(result.count,1);
}
} // namespace
