// SPDX-License-Identifier: MIT
// Qualification of the existing SAP as a tied-search candidate provider.
// This is not original-deck bucket traversal or a runtime attachment owner.
#include "NativeOracle.h"
#include "BoundsFixture.h"
#include "lib_src/collision/HydroelasticBroadphase.cuh"
#include <set>
#include <stdexcept>

namespace tied_search_test {
namespace {
using Pair = std::pair<int, int>;  // master rank, secondary rank

std::vector<ts::Vec3> Points() {
  std::vector<ts::Vec3> out;
  for (int x=-9; x<=9; ++x)
    for (int y=-9; y<=9; ++y)
      for (double z : {-0.0015, -0.0007, 0., 0.0007, 0.0015})
        out.push_back({x*0.0015, y*0.0015, z});
  return out;
}

// Borrowed inflation storage must outlive Broadphase, including queued work.
struct DeviceRadii {
  double* data=nullptr;
  explicit DeviceRadii(const std::vector<double>& radii) {
    if (cudaMalloc(&data, radii.size()*sizeof(double))!=cudaSuccess)
      throw std::runtime_error("candidate radius allocation failed");
    if (cudaMemcpy(data, radii.data(), radii.size()*sizeof(double),
                   cudaMemcpyHostToDevice)!=cudaSuccess) {
      cudaFree(data);
      throw std::runtime_error("candidate radius upload failed");
    }
  }
  ~DeviceRadii() { cudaFree(data); }
  DeviceRadii(const DeviceRadii&)=delete;
  DeviceRadii& operator=(const DeviceRadii&)=delete;
};

struct Scene {
  std::vector<ts::SearchInput> masters;
  std::vector<ts::Vec3> secondary=Points();
  Eigen::MatrixXd nodes;
  Eigen::MatrixXi elements;
  Eigen::VectorXi bodies;
  std::vector<double> radii;

  Scene() {
    // Duplicate rank 0 at rank 3 exercises first-in-order exact ties.
    masters={Shape(0), Shape(1), Shape(2), Shape(0)};
    auto unequal=Shape();unequal.geometry_m.master_position[0].x=-.1;
    masters.push_back(unequal);
    auto degenerate=Shape();
    for(auto& p:degenerate.geometry_m.master_position) p={};
    masters.push_back(degenerate);
    const int m=masters.size(), count=m+secondary.size();
    nodes.resize(4*m+secondary.size(), 3);
    elements.resize(count, 4);
    bodies.resize(count);
    radii.resize(count, 0.);
    for (int i=0; i<m; ++i) {
      ts::NativeSearchBounds bounds;
      if (ts::PrepareSearchBounds(BoundsInput(masters[i]), bounds)!=ts::Status::Success)
        throw std::runtime_error("invalid test search master");
      // The startup search uses original working coordinates. Its MAX-diagonal
      // radius differs from the MIN-diagonal final projection gap.
      radii[i]=std::nextafter(bounds.inflation,
                             std::numeric_limits<double>::infinity());
      for (int k=0; k<4; ++k) {
        const auto p=masters[i].geometry_m.master_position[k];
        nodes.row(4*i+k)<<p.x/masters[i].working_length_to_m,
            p.y/masters[i].working_length_to_m,p.z/masters[i].working_length_to_m;
        elements(i,k)=4*i+k;
      }
      bodies[i]=0;
    }
    for (int i=0; i<static_cast<int>(secondary.size()); ++i) {
      const auto p=secondary[i];
      nodes.row(4*m+i)<<p.x/masters[0].working_length_to_m,
          p.y/masters[0].working_length_to_m,p.z/masters[0].working_length_to_m;
      elements.row(m+i).setConstant(4*m+i);
      bodies[m+i]=1;
    }
  }
};

void Initialize(const Scene& s, const DeviceRadii& radii, Broadphase& bp) {
  bp.Initialize(s.nodes,s.elements,s.bodies);
  bp.EnableSelfCollision(false);
  BroadphaseAABBOptions options;
  options.d_elementInflation=radii.data;
  bp.SetAABBOptions(options);
  bp.SetDetectionLimits({32768, 4*1024*1024});
  bp.CreateAABB();
}

std::set<Pair> Candidates(const Scene& s, const Broadphase& bp) {
  std::set<Pair> result;
  const int m=s.masters.size();
  for (const auto pair : bp.h_collisionPairs) {
    const int a=std::min(pair.idA,pair.idB), b=std::max(pair.idA,pair.idB);
    EXPECT_GE(a,0); EXPECT_LT(a,m);
    EXPECT_GE(b,m); EXPECT_LT(b,m+static_cast<int>(s.secondary.size()));
    result.emplace(a,b-m);
  }
  EXPECT_EQ(result.size(),bp.h_collisionPairs.size());
  return result;
}

class TiedSearchBroadphase : public ::testing::Test {
  void SetUp() override {
    int devices=0;
    ASSERT_EQ(cudaGetDeviceCount(&devices),cudaSuccess);
    ASSERT_GT(devices,0) << "Actual CUDA required; this gate must not skip";
  }
};

TEST_F(TiedSearchBroadphase, AllAxesRetainNativeAcceptedPairsAndOrderedChoices) {
  Scene scene;
  DeviceRadii radii(scene.radii);
  Broadphase bp;
  Initialize(scene,radii,bp);
  std::set<Pair> accepted;
  std::vector<NativeChoice> expected(scene.secondary.size());
  for (int i=0; i<static_cast<int>(scene.secondary.size()); ++i)
    for (int m=0; m<static_cast<int>(scene.masters.size()); ++m) {
      auto input=scene.masters[m];
      input.geometry_m.secondary_position=scene.secondary[i];
      if(NativeBounds(BoundsInput(input),scene.secondary[i])[7]!=1) continue;
      if (Native(input,m+1,expected[i]).admissible) accepted.emplace(m,i);
    }
  ASSERT_FALSE(accepted.empty());
  for (int axis=0; axis<3; ++axis) {
    SCOPED_TRACE(axis);
    bp.SortAABBs(axis);
    bp.DetectCollisions(true);
    const auto pairs=Candidates(scene,bp);
    for (const auto pair : accepted) ASSERT_EQ(pairs.count(pair),1u);
    // Never consume SAP order as native rank order. Explicitly restore it.
    std::vector<ts::SearchChoice> actual(scene.secondary.size());
    for (const auto [m,i] : pairs) {
      auto input=scene.masters[m];
      input.geometry_m.secondary_position=scene.secondary[i];
      ts::NativeSearchBounds bounds;bool within=false;
      ASSERT_EQ(ts::PrepareSearchBounds(BoundsInput(input),bounds),ts::Status::Success);
      ASSERT_EQ(ts::WithinSearchBounds(bounds,scene.secondary[i],within),ts::Status::Success);
      if(!within) continue;
      ASSERT_EQ(ts::ConsiderCandidate(input,m+1,actual[i]),ts::Status::Success);
    }
    for (std::size_t i=0; i<actual.size(); ++i) {
      SCOPED_TRACE(i);
      ASSERT_EQ(actual[i].matched,expected[i].selected!=0);
      if (!actual[i].matched) continue;
      ASSERT_EQ(actual[i].ordered_master,expected[i].selected);
      EXPECT_NEAR(actual[i].projection.s,expected[i].st[0],1e-12);
      EXPECT_NEAR(actual[i].projection.t,expected[i].st[1],1e-12);
      EXPECT_NEAR(actual[i].projection.selection_distance,expected[i].distance,1e-12);
      ASSERT_FALSE(HasFailure());
    }
  }
}

TEST_F(TiedSearchBroadphase, PairBudgetFailurePublishesNothingAndAllowsExactRetry) {
  Scene scene;
  DeviceRadii radii(scene.radii);
  Broadphase bp;
  Initialize(scene,radii,bp);
  bp.SortAABBs(0);
  bp.DetectCollisions(true);
  const auto expected=Candidates(scene,bp);
  ASSERT_GT(expected.size(),1u);
  bp.SetDetectionLimits({expected.size()-1,4*1024*1024});
  EXPECT_THROW(bp.DetectCollisions(true),std::length_error);
  EXPECT_EQ(bp.numCollisions,0);
  EXPECT_TRUE(bp.h_collisionPairs.empty());
  EXPECT_EQ(bp.GetCollisionPairsDevicePtr(),nullptr);
  bp.SetDetectionLimits({expected.size(),4*1024*1024});
  bp.DetectCollisions(true);
  EXPECT_EQ(Candidates(scene,bp),expected);
}
} // namespace
} // namespace tied_search_test
