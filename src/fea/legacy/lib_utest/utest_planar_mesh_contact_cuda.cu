#include "lib_src/collision/PlanarMeshContact.h"
#include "lib_src/solvers/ExplicitTranslationStep.h"
#include <gtest/gtest.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <vector>

namespace {
namespace sc = tlfea::contact;
namespace fea = tl::fea;
using PS = sc::PlanarContactStatus;
constexpr std::uint64_t LargeId = std::uint64_t{1} << 54;

struct Wall {
  std::vector<sc::PlanarWallVertex> vertices;
  std::vector<sc::PlanarWallTriangle> triangles;
  sc::PlanarWallView view() const {
    return {vertices.data(),static_cast<std::uint32_t>(vertices.size()),
            triangles.data(),static_cast<std::uint32_t>(triangles.size())};
  }
};
Wall SquareWall(bool subdivided = false) {
  Wall w;
  w.vertices = {{{0,-2,-2},1,LargeId+1},{{0,2,-2},2,LargeId+2},
                {{0,2,2},3,LargeId+3},{{0,-2,2},4,LargeId+4}};
  if (!subdivided) w.triangles = {{{0,2,1},LargeId+9,71,LargeId+71},{{0,3,2},LargeId+5,71,LargeId+71}};
  else {
    w.vertices.push_back({{0,0,0},5,LargeId+5});
    w.triangles = {{{0,4,1},LargeId+14,71,LargeId+71},{{1,4,2},LargeId+13,71,LargeId+71},
                   {{2,4,3},LargeId+12,71,LargeId+71},{{3,4,0},LargeId+11,71,LargeId+71}};
  }
  return w;
}
struct Rig {
  std::vector<sc::PlanarSurfaceNode> nodes;
  std::vector<sc::PlanarSurfaceTriangle> triangles;
  std::array<double,75> x{},v{};
  std::array<double,25> inverse{};
  std::array<std::uint8_t,25> fixed{};
  sc::PlanarSurfaceView surface() const {
    return {nodes.data(),static_cast<std::uint32_t>(nodes.size()),triangles.data(),
            static_cast<std::uint32_t>(triangles.size())};
  }
  void Current(double depth, double speed) {
    for (const auto& node : nodes) {
      const auto i = node.global_node;
      x[3*i] = depth; x[3*i+1] = node.reference_position.y; x[3*i+2] = node.reference_position.z;
      v[3*i] = speed; v[3*i+1] = v[3*i+2] = 0; inverse[i] = 1;
    }
  }
  fea::NodalReport Initialize(fea::FENodalState& owner, double h = .001) const {
    fea::NodalStateConfig c; c.node_count = nodes.size(); c.fixed_dt = h;
    return owner.Initialize(c,{x.data(),v.data(),nullptr,nodes.size()},inverse.data(),fixed.data());
  }
  sc::PlanarContactConfig config(const fea::FENodalState& owner) const {
    sc::PlanarContactConfig c; c.owner_id = owner.accepted().owner_id;
    c.global_node_count = static_cast<std::uint32_t>(nodes.size());
    c.penalty_per_area = 1200; c.max_penetration_m = .1; return c;
  }
};
Rig SquareRig(bool refined = false) {
  Rig r;
  if (!refined) {
    r.nodes = {{3,{0,-.1,-.1},LargeId+103},{0,{0,.1,-.1},LargeId+100},
                {2,{0,.1,.1},LargeId+102},{1,{0,-.1,.1},LargeId+101}};
    r.triangles = {{{0,1,2},101},{{0,2,3},102}};
  } else {
    for (unsigned j = 0; j < 3; ++j) for (unsigned i = 0; i < 3; ++i) {
      const unsigned n = 3*j+i;
      r.nodes.push_back({n,{0,-.1+.1*i,-.1+.1*j},LargeId+100+n});
    }
    for (unsigned j = 0; j < 2; ++j) for (unsigned i = 0; i < 2; ++i) {
      const unsigned n = 3*j+i;
      r.triangles.push_back({{n,n+1,n+4},101+2*n});
      r.triangles.push_back({{n,n+4,n+3},102+2*n});
    }
  }
  r.Current(.01,1); return r;
}
Rig SeamRig() {
  Rig r;
  r.nodes = {{0,{0,-.1,-.1},LargeId+100},{1,{0,.1,-.1},LargeId+101},{2,{0,0,.2},LargeId+102}};
  r.triangles = {{{0,1,2},101}};
  r.Current(.01,1); return r;
}

__global__ void SeedForce(fea::NodalAssemblyView view, double value, unsigned node = UINT32_MAX) {
  if (node == UINT32_MAX)
    for (unsigned i = 0; i < view.accepted.node_count; ++i) view.forces.force_x[i] += value;
  else view.forces.force_x[node] += value;
}
__global__ void Brake(fea::NodalAssemblyView view, double h) {
  for (unsigned i = 0; i < view.accepted.node_count; ++i)
    view.forces.force_x[i] -= view.accepted.velocity_xyz[3*i]/(h*view.mass.inverse_mass[i]);
}
__global__ void TangentialForce(fea::NodalAssemblyView view) { view.forces.force_y[0] += 1; }

std::array<double,25> ReadForces(const fea::NodalAssemblyView& view) {
  std::array<double,25> result{};
  EXPECT_EQ(cudaMemcpyAsync(result.data(),view.forces.force_x,
                            view.accepted.node_count*sizeof(double),cudaMemcpyDeviceToHost,view.stream),cudaSuccess);
  EXPECT_EQ(cudaStreamSynchronize(view.stream),cudaSuccess);
  return result;
}
void SameAccepted(const Rig& initial, fea::FENodalState& owner, std::uint64_t epoch = 0) {
  std::array<double,75> x{},v{}; fea::NodalStamp stamp;
  ASSERT_EQ(owner.CopyAccepted({x.data(),v.data(),initial.nodes.size()},&stamp).status,fea::NodalStatus::Ok);
  EXPECT_EQ(stamp.epoch,epoch);
  for (unsigned i = 0; i < initial.nodes.size()*3; ++i) {
    EXPECT_DOUBLE_EQ(x[i],initial.x[i]); EXPECT_DOUBLE_EQ(v[i],initial.v[i]);
  }
}
class PlanarContact : public ::testing::Test {
  void SetUp() override {
    int count = 0; ASSERT_EQ(cudaGetDeviceCount(&count),cudaSuccess); ASSERT_GT(count,0);
  }
};

TEST_F(PlanarContact, IndependentPressureQuadratureSharedScatterMomentAndPower) {
  auto rig = SquareRig(); const auto wall = SquareWall();
  const double depth[4] = {.01,.02,.03,.04}, speed[4] = {1,2,3,4};
  for (int i = 0; i < 4; ++i) {
    const auto n = rig.nodes[i].global_node;
    rig.x[3*n] = depth[i]; rig.v[3*n] = speed[i]; rig.inverse[n] = 1.0/(i+1);
  }
  fea::FENodalState owner; ASSERT_EQ(rig.Initialize(owner).status,fea::NodalStatus::Ok);
  sc::PlanarMeshContact batch; ASSERT_EQ(batch.Initialize(rig.config(owner),wall.view(),rig.surface()).status,PS::Ok);
  const auto allocation = batch.allocations();
  fea::NodalTrialToken token; fea::NodalAssemblyView view;
  ASSERT_EQ(owner.BeginTrial(&token,&view).status,fea::NodalStatus::Ok);
  SeedForce<<<1,1,0,view.stream>>>(view,.125); ASSERT_EQ(cudaGetLastError(),cudaSuccess);
  ASSERT_EQ(batch.Evaluate(view).status,PS::Ok); ASSERT_NE(batch.diagnostics(),nullptr);
  const auto& d = *batch.diagnostics(); const auto force = ReadForces(view);
  // Each triangle has area .02 m². Integrating a constant centroid pressure
  // gives F1=.48N, F2=.64N. Each linear nodal basis integrates to area/3.
  const double expected[4] = {-(.48+.64)/3,-.48/3,-(.48+.64)/3,-.64/3};
  double work = 0; sc::Vec3 moment{};
  for (int i = 0; i < 4; ++i) {
    const auto n = rig.nodes[i].global_node;
    EXPECT_NEAR(force[n]-.125,expected[i],2e-13);
    work += (force[n]-.125)*speed[i];
    moment.y += rig.x[3*n+2]*(force[n]-.125);
    moment.z -= rig.x[3*n+1]*(force[n]-.125);
  }
  EXPECT_NEAR(d.wall_reaction.x,1.12,2e-13);
  EXPECT_NEAR(d.wall_moment.y,.016/3,2e-13); EXPECT_NEAR(d.wall_moment.z,.016/3,2e-13);
  EXPECT_NEAR(moment.y+d.wall_moment.y,0,2e-13); EXPECT_NEAR(moment.z+d.wall_moment.z,0,2e-13);
  EXPECT_NEAR(d.surface_power,-8.0/3,2e-13); EXPECT_NEAR(work,d.surface_power,2e-13);
  EXPECT_NEAR(d.elastic_energy,1.0/75,2e-13);
  EXPECT_EQ(d.owner_id,owner.accepted().owner_id); EXPECT_EQ(d.base_epoch,0); EXPECT_EQ(d.attempt,view.attempt);
  EXPECT_EQ(d.sample_count,2); EXPECT_EQ(d.covered_count,2); EXPECT_EQ(d.active_count,2);
  EXPECT_EQ(batch.allocations().device_bytes,allocation.device_bytes);
  EXPECT_EQ(batch.allocations().device_allocations,4);
  ASSERT_EQ(owner.SealAssembly(token).status,fea::NodalStatus::Ok);
  owner.Discard(); SameAccepted(rig,owner);
}

TEST_F(PlanarContact, InternalSeamPermutationAndSubdivisionKeepOneSpring) {
  const auto rig = SeamRig();
  for (int variant = 0; variant < 3; ++variant) {
    SCOPED_TRACE(variant);
    auto wall = SquareWall(variant == 2);
    if (variant == 1) std::reverse(wall.triangles.begin(),wall.triangles.end());
    fea::FENodalState owner; ASSERT_EQ(rig.Initialize(owner).status,fea::NodalStatus::Ok);
    sc::PlanarMeshContact batch; ASSERT_EQ(batch.Initialize(rig.config(owner),wall.view(),rig.surface()).status,PS::Ok);
    fea::NodalTrialToken token; fea::NodalAssemblyView view;
    ASSERT_EQ(owner.BeginTrial(&token,&view).status,fea::NodalStatus::Ok);
    ASSERT_EQ(batch.Evaluate(view).status,PS::Ok);
    const auto& d = *batch.diagnostics();
    EXPECT_EQ(d.sample_count,1); EXPECT_EQ(d.covered_count,1);
    EXPECT_NEAR(d.wall_reaction.x,.36,1e-13); EXPECT_NEAR(d.elastic_energy,.0018,1e-13);
    EXPECT_EQ(d.samples[0].wall_triangle_id,LargeId+(variant == 2 ? 11 : 5));
    EXPECT_EQ(d.samples[0].wall_source_quad_id,71);
    EXPECT_EQ(d.samples[0].wall_assembled_source_quad_id,LargeId+71);
    EXPECT_EQ(d.samples[0].wall_feature.kind,variant == 2 ? sc::FeatureKind::kVertex : sc::FeatureKind::kEdge);
    const auto force = ReadForces(view);
    for (int i = 0; i < 3; ++i) EXPECT_NEAR(force[i],-.12,1e-13);
    owner.Discard();
  }
}

TEST_F(PlanarContact, SurfaceAreaRefinementPreservesIntegratedResponse) {
  const auto wall = SquareWall();
  for (bool refined : {false,true}) {
    auto rig = SquareRig(refined); fea::FENodalState owner;
    ASSERT_EQ(rig.Initialize(owner).status,fea::NodalStatus::Ok);
    sc::PlanarMeshContact batch; ASSERT_EQ(batch.Initialize(rig.config(owner),wall.view(),rig.surface()).status,PS::Ok);
    fea::NodalTrialToken token; fea::NodalAssemblyView view;
    ASSERT_EQ(owner.BeginTrial(&token,&view).status,fea::NodalStatus::Ok);
    ASSERT_EQ(batch.Evaluate(view).status,PS::Ok);
    EXPECT_NEAR(batch.diagnostics()->wall_reaction.x,.48,2e-13);
    EXPECT_NEAR(batch.diagnostics()->elastic_energy,.0024,2e-13);
    EXPECT_NEAR(batch.diagnostics()->surface_power,-.48,2e-13);
    const auto force = ReadForces(view); double sum = 0;
    for (unsigned i = 0; i < rig.nodes.size(); ++i) sum += force[i];
    EXPECT_NEAR(sum,-.48,2e-13); owner.Discard();
  }
}

TEST_F(PlanarContact, FiniteFootprintMissRemainsForceFreeBeyondPlanePenetrationCap) {
  auto rig = SquareRig(); for (auto& n : rig.nodes) n.reference_position.y += 5;
  rig.Current(10,1); const auto wall = SquareWall();
  fea::FENodalState owner; ASSERT_EQ(rig.Initialize(owner).status,fea::NodalStatus::Ok);
  sc::PlanarMeshContact batch; ASSERT_EQ(batch.Initialize(rig.config(owner),wall.view(),rig.surface()).status,PS::Ok);
  fea::NodalTrialToken token; fea::NodalAssemblyView view;
  ASSERT_EQ(owner.BeginTrial(&token,&view).status,fea::NodalStatus::Ok);
  ASSERT_EQ(batch.Evaluate(view).status,PS::Ok);
  EXPECT_EQ(batch.diagnostics()->covered_count,0); EXPECT_EQ(batch.diagnostics()->active_count,0);
  EXPECT_DOUBLE_EQ(batch.diagnostics()->wall_reaction.x,0);
  for (double force : ReadForces(view)) EXPECT_DOUBLE_EQ(force,0);
  ASSERT_EQ(owner.SealAssembly(token).status,fea::NodalStatus::Ok);
  ASSERT_EQ(fea::AdvanceTranslations(owner,token).status,fea::NodalStatus::Ok);
  fea::NodalPreparedView prepared; ASSERT_EQ(owner.BorrowPrepared(token,&prepared).status,fea::NodalStatus::Ok);
  ASSERT_EQ(batch.ValidatePrepared(prepared).status,PS::Ok);
  ASSERT_EQ(owner.Commit(token).status,fea::NodalStatus::Ok);
}

TEST_F(PlanarContact, InactiveCoveredSpringsStillContributeTheFrozenStepBound) {
  auto rig = SquareRig(); rig.Current(-.05,0); const auto wall = SquareWall();
  fea::FENodalState owner; ASSERT_EQ(rig.Initialize(owner,1).status,fea::NodalStatus::Ok);
  sc::PlanarMeshContact batch; ASSERT_EQ(batch.Initialize(rig.config(owner),wall.view(),rig.surface()).status,PS::Ok);
  fea::NodalTrialToken token; fea::NodalAssemblyView view;
  ASSERT_EQ(owner.BeginTrial(&token,&view).status,fea::NodalStatus::Ok);
  ASSERT_EQ(batch.Evaluate(view).status,PS::Ok);
  EXPECT_EQ(batch.diagnostics()->covered_count,2); EXPECT_EQ(batch.diagnostics()->active_count,0);
  EXPECT_DOUBLE_EQ(batch.diagnostics()->wall_reaction.x,0);
  // K has centroid stencils k=24 and unit nodal masses. Its largest absolute
  // row sum is 16/s², so safety .8 limits h to .4s, even before impact.
  const auto report = owner.SealAssembly(token);
  EXPECT_EQ(report.status,fea::NodalStatus::StepTooLarge);
  EXPECT_NEAR(report.stable_dt,.4,1e-12); SameAccepted(rig,owner);
}

TEST_F(PlanarContact, PreparedVertexDepthAndTangentialMotionRejectBeforeCommitThenRetry) {
  const auto wall = SquareWall();
  for (bool tangent : {false,true}) {
    auto rig = SquareRig(); rig.Current(0,0);
    if (!tangent) rig.v[3*rig.nodes[3].global_node] = 3;
    fea::FENodalState owner; ASSERT_EQ(rig.Initialize(owner,.01).status,fea::NodalStatus::Ok);
    auto config = rig.config(owner); config.max_penetration_m = .015;
    sc::PlanarMeshContact batch; ASSERT_EQ(batch.Initialize(config,wall.view(),rig.surface()).status,PS::Ok);
    fea::NodalTrialToken token; fea::NodalAssemblyView view;
    ASSERT_EQ(owner.BeginTrial(&token,&view).status,fea::NodalStatus::Ok);
    ASSERT_EQ(batch.Evaluate(view).status,PS::Ok);
    if (tangent) { TangentialForce<<<1,1,0,view.stream>>>(view); ASSERT_EQ(cudaGetLastError(),cudaSuccess); }
    ASSERT_EQ(owner.SealAssembly(token).status,fea::NodalStatus::Ok);
    ASSERT_EQ(fea::AdvanceTranslations(owner,token).status,fea::NodalStatus::Ok);
    fea::NodalPreparedView prepared; ASSERT_EQ(owner.BorrowPrepared(token,&prepared).status,fea::NodalStatus::Ok);
    // In the depth case, the centroid reaches only .01m, but a vertex reaches
    // .03m. Checking only force-sample depth would incorrectly accept it.
    EXPECT_EQ(batch.ValidatePrepared(prepared).status,PS::UnsupportedMotion);
    EXPECT_EQ(batch.diagnostics(),nullptr); owner.Discard(); SameAccepted(rig,owner);
    ASSERT_EQ(owner.BeginTrial(&token,&view).status,fea::NodalStatus::Ok);
    ASSERT_EQ(batch.Evaluate(view).status,PS::Ok);
    Brake<<<1,1,0,view.stream>>>(view,.01); ASSERT_EQ(cudaGetLastError(),cudaSuccess);
    ASSERT_EQ(owner.SealAssembly(token).status,fea::NodalStatus::Ok);
    ASSERT_EQ(fea::AdvanceTranslations(owner,token).status,fea::NodalStatus::Ok);
    ASSERT_EQ(owner.BorrowPrepared(token,&prepared).status,fea::NodalStatus::Ok);
    ASSERT_EQ(batch.ValidatePrepared(prepared).status,PS::Ok);
    EXPECT_EQ(batch.diagnostics()->base_epoch,0); // Still the force evaluation epoch.
    ASSERT_EQ(owner.Commit(token).status,fea::NodalStatus::Ok); EXPECT_EQ(owner.accepted().epoch,1);
  }
}

TEST_F(PlanarContact, FiniteLateAssemblyOverflowInvalidatesTrialAndCleanRetryWorks) {
  auto rig = SeamRig();
  rig.nodes[0].reference_position = {0,-1,-.5}; rig.nodes[1].reference_position = {0,1,-.5};
  rig.nodes[2].reference_position = {0,0,.5}; rig.Current(1,0);
  for (int i = 0; i < 3; ++i) rig.inverse[i] = 1e-308;
  fea::FENodalState owner; ASSERT_EQ(rig.Initialize(owner).status,fea::NodalStatus::Ok);
  auto config = rig.config(owner); config.penalty_per_area = 1e308; config.max_penetration_m = 2;
  const auto wall = SquareWall(); sc::PlanarMeshContact batch;
  ASSERT_EQ(batch.Initialize(config,wall.view(),rig.surface()).status,PS::Ok);
  fea::NodalTrialToken token; fea::NodalAssemblyView view;
  ASSERT_EQ(owner.BeginTrial(&token,&view).status,fea::NodalStatus::Ok);
  SeedForce<<<1,1,0,view.stream>>>(view,-1.6e308,2); ASSERT_EQ(cudaGetLastError(),cudaSuccess);
  // Contact force -1e308N and its energy remain finite; the last additive
  // nodal write -1.6e308 - 1e308/3 overflows. Earlier writes must stay staged.
  EXPECT_EQ(batch.Evaluate(view).status,PS::InvalidOutput); EXPECT_EQ(batch.diagnostics(),nullptr);
  const auto failed = ReadForces(view);
  EXPECT_DOUBLE_EQ(failed[0],0); EXPECT_DOUBLE_EQ(failed[1],0); EXPECT_DOUBLE_EQ(failed[2],-1.6e308);
  EXPECT_EQ(owner.SealAssembly(token).status,fea::NodalStatus::ContributorFailure);
  SameAccepted(rig,owner);
  ASSERT_EQ(owner.BeginTrial(&token,&view).status,fea::NodalStatus::Ok);
  ASSERT_EQ(batch.Evaluate(view).status,PS::Ok);
  EXPECT_NEAR(batch.diagnostics()->wall_reaction.x/1e308,1,1e-12);
  ASSERT_EQ(owner.SealAssembly(token).status,fea::NodalStatus::Ok);
  ASSERT_EQ(fea::AdvanceTranslations(owner,token).status,fea::NodalStatus::Ok);
  fea::NodalPreparedView prepared; ASSERT_EQ(owner.BorrowPrepared(token,&prepared).status,fea::NodalStatus::Ok);
  ASSERT_EQ(batch.ValidatePrepared(prepared).status,PS::Ok);
  ASSERT_EQ(owner.Commit(token).status,fea::NodalStatus::Ok);
}

TEST_F(PlanarContact, DuplicateAndWrongOwnerContributionsFailClosed) {
  auto rig = SquareRig(); const auto wall = SquareWall(); fea::FENodalState a,b;
  ASSERT_EQ(rig.Initialize(a).status,fea::NodalStatus::Ok); ASSERT_EQ(rig.Initialize(b).status,fea::NodalStatus::Ok);
  sc::PlanarMeshContact batch; ASSERT_EQ(batch.Initialize(rig.config(a),wall.view(),rig.surface()).status,PS::Ok);
  fea::NodalTrialToken token; fea::NodalAssemblyView view;
  ASSERT_EQ(b.BeginTrial(&token,&view).status,fea::NodalStatus::Ok);
  EXPECT_EQ(batch.Evaluate(view).status,PS::WrongOwner);
  EXPECT_EQ(b.SealAssembly(token).status,fea::NodalStatus::ContributorFailure);
  ASSERT_EQ(a.BeginTrial(&token,&view).status,fea::NodalStatus::Ok);
  ASSERT_EQ(batch.Evaluate(view).status,PS::Ok);
  const auto force = ReadForces(view);
  EXPECT_EQ(batch.Evaluate(view).status,PS::StaleAttempt); EXPECT_EQ(batch.diagnostics(),nullptr);
  EXPECT_EQ(ReadForces(view),force);
  EXPECT_EQ(a.SealAssembly(token).status,fea::NodalStatus::ContributorFailure);
  SameAccepted(rig,a); SameAccepted(rig,b);
  ASSERT_EQ(a.BeginTrial(&token,&view).status,fea::NodalStatus::Ok);
  ASSERT_EQ(batch.Evaluate(view).status,PS::Ok); EXPECT_EQ(ReadForces(view),force); a.Discard();
}

TEST_F(PlanarContact, InvalidTopologyBoundaryMotionAndBudgetAreExplicit) {
  auto rig = SquareRig(); auto wall = SquareWall(); fea::FENodalState owner;
  ASSERT_EQ(rig.Initialize(owner).status,fea::NodalStatus::Ok); auto config = rig.config(owner);
  sc::PlanarMeshContact budget; auto small = config; small.max_device_bytes = 1;
  EXPECT_EQ(budget.Initialize(small,wall.view(),rig.surface()).status,PS::ResourceLimit);
  EXPECT_EQ(budget.allocations().device_bytes,0);
  ASSERT_EQ(budget.Initialize(config,wall.view(),rig.surface()).status,PS::Ok);
  EXPECT_LE(budget.allocations().device_bytes,sc::MaxPlanarContactDeviceBytes);
  { auto bad = wall; bad.triangles[0].nodes[0] = 100;
    sc::PlanarMeshContact c; EXPECT_EQ(c.Initialize(config,bad.view(),rig.surface()).status,PS::InvalidInput); }
  { auto bad = wall; std::swap(bad.triangles[0].nodes[1],bad.triangles[0].nodes[2]);
    sc::PlanarMeshContact c; EXPECT_EQ(c.Initialize(config,bad.view(),rig.surface()).status,PS::UnsupportedGeometry); }
  { auto bad = wall; bad.vertices[2].position.x = 1e-15;
    sc::PlanarMeshContact c; EXPECT_EQ(c.Initialize(config,bad.view(),rig.surface()).status,PS::UnsupportedGeometry); }
  { auto bad = wall; bad.vertices[0].position.y = std::numeric_limits<double>::quiet_NaN();
    sc::PlanarMeshContact c; EXPECT_EQ(c.Initialize(config,bad.view(),rig.surface()).status,PS::UnsupportedGeometry); }
  { auto bad = rig; for (auto& n : bad.nodes) n.reference_position.y += 2;
    sc::PlanarMeshContact c; EXPECT_EQ(c.Initialize(config,wall.view(),bad.surface()).status,PS::AmbiguousBoundary); }
  { auto bad = wall; bad.vertices.push_back({{0,-.1,-.1},5,LargeId+5});
    bad.vertices.push_back({{0,.1,-.1},6,LargeId+6}); bad.vertices.push_back({{0,0,.1},7,LargeId+7});
    bad.triangles.push_back({{4,6,5},25,71,LargeId+71});
    sc::PlanarMeshContact c; EXPECT_EQ(c.Initialize(config,bad.view(),rig.surface()).status,PS::UnsupportedGeometry); }
  { auto bad = rig; bad.v[1] = .01; fea::FENodalState moving;
    ASSERT_EQ(bad.Initialize(moving).status,fea::NodalStatus::Ok); sc::PlanarMeshContact c;
    ASSERT_EQ(c.Initialize(bad.config(moving),wall.view(),bad.surface()).status,PS::Ok);
    fea::NodalTrialToken token; fea::NodalAssemblyView view;
    ASSERT_EQ(moving.BeginTrial(&token,&view).status,fea::NodalStatus::Ok);
    EXPECT_EQ(c.Evaluate(view).status,PS::UnsupportedMotion);
    EXPECT_EQ(moving.SealAssembly(token).status,fea::NodalStatus::ContributorFailure); SameAccepted(bad,moving); }
}
}  // namespace
