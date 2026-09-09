// Actual force-driven TL state -> Chrono mesh integration. Three free physical
// nodes carry explicit masses; no shell element, contact or Chrono dynamics.
#include "NodalMeshOutput.h"
#include "lib_src/solvers/ExplicitTranslationStep.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"
#include <gtest/gtest.h>
#include <array>
#include <limits>

namespace {
namespace fea = tl::fea;
namespace visual = crash::visual;

__global__ void AddAcceleration(fea::NodalAssemblyView view, double acceleration) {
    if (blockIdx.x || threadIdx.x) return;
    for (std::size_t i = 0; i < view.accepted.node_count; ++i)
        view.forces.force_x[i] += acceleration / view.mass.inverse_mass[i];
}
__global__ void FiniteForceCausingAdvanceOverflow(fea::NodalAssemblyView view) {
    if (blockIdx.x || threadIdx.x) return;
    view.forces.force_x[0] = .25;  // Earlier candidate nodes move before failure.
    view.forces.force_x[1] = .5;
    view.forces.force_x[2] = 1.7e308;  // Finite force; inverse mass 4 overflows v'.
}

struct Case {
    std::array<double, 9> x{{0,0,0, 0,1,0, 0,0,1}}, v{};
    std::array<double, 3> inverse{{1, .5, 1.0/3}};
    std::array<std::uint8_t, 3> fixed{};
    fea::NodalReport Initialize(fea::FENodalState& state, double h = .01) const {
        fea::NodalStateConfig c; c.node_count=3; c.fixed_dt=h;
        return state.Initialize(c, {x.data(),v.data(),nullptr,3}, inverse.data(), fixed.data());
    }
};

visual::Binding Binding(const fea::FENodalState& owner) {
    visual::Binding b; b.identity={owner.accepted().owner_id,7,11}; b.tl_node_count=3;
    for (auto i : {2u,0u,1u}) b.vertices.push_back({i,{31,2,(std::uint64_t{1}<<54)+i+1}});
    b.triangles={{{1,2,0},31,2,101,5,0,0}};
    return b;
}
void CheckMesh(const visual::NodalMeshOutput& output, double displacement, std::uint64_t epoch) {
    const auto& mesh=output.surface();
    ASSERT_NE(mesh.frame(),nullptr); ASSERT_NE(mesh.mesh(),nullptr);
    EXPECT_EQ(mesh.frame()->epoch,epoch);
    const auto& vertices=mesh.mesh()->GetCoordsVertices();
    const Case initial;
    for (std::size_t i=0;i<3;++i) {
        const auto n=mesh.binding()->vertices[i].tl_node;
        EXPECT_NEAR(vertices[i].x(),displacement,2e-13);
        EXPECT_DOUBLE_EQ(vertices[i].y(),initial.x[3*n+1]);
        EXPECT_DOUBLE_EQ(vertices[i].z(),initial.x[3*n+2]);
        EXPECT_EQ(mesh.binding()->vertices[i].source.node,(std::uint64_t{1}<<54)+n+1);
    }
}
class NodalOutput:public ::testing::Test {
 protected:
    void SetUp() override {
        int count=0; ASSERT_EQ(cudaGetDeviceCount(&count),cudaSuccess); ASSERT_GT(count,0);
    }
};

TEST_F(NodalOutput, ActualGpuForceMotionPublishesOnlyAcceptedOutputCadence) {
    Case input; fea::FENodalState owner;
    ASSERT_EQ(input.Initialize(owner).status,fea::NodalStatus::Ok);
    visual::NodalMeshOutput output;
    ASSERT_EQ(output.Initialize(owner,Binding(owner)).status,visual::Status::Ok);
    ASSERT_EQ(output.Publish(owner).status,visual::Status::Ok);
    const auto allocation=owner.allocations();
    for (std::uint64_t n=1;n<=10;++n) {
        fea::NodalTrialToken token; fea::NodalAssemblyView view;
        ASSERT_EQ(owner.BeginTrial(&token,&view).status,fea::NodalStatus::Ok);
        AddAcceleration<<<1,1,0,view.stream>>>(view,.5);
        AddAcceleration<<<1,1,0,view.stream>>>(view,1.5);
        ASSERT_EQ(cudaGetLastError(),cudaSuccess);
        ASSERT_EQ(owner.SealAssembly(token).status,fea::NodalStatus::Ok);
        ASSERT_EQ(fea::AdvanceTranslations(owner,token).status,fea::NodalStatus::Ok);
        if (n==1) { // A completed candidate still has no publication authority.
            EXPECT_EQ(output.Publish(owner).status,visual::Status::StaleFrame);
            CheckMesh(output,0,0);
        }
        const auto shown=output.surface().frame()->epoch;
        ASSERT_EQ(owner.Commit(token).status,fea::NodalStatus::Ok);
        EXPECT_EQ(output.surface().frame()->epoch,shown);
        if (n%5==0) {
            ASSERT_EQ(output.Publish(owner).status,visual::Status::Ok);
            // Independent sum of n constant acceleration impulses. Velocity-first
            // positions differ from continuous x=0.5*a*t^2 by 0.5*a*t*h.
            const double h=.01, t=n*h;
            CheckMesh(output,h*h*n*(n+1),n);
            EXPECT_NEAR(output.surface().mesh()->GetCoordsVertices()[0].x()-t*t,t*h,2e-13);
        }
    }
    EXPECT_EQ(owner.allocations().device_bytes,allocation.device_bytes);
    EXPECT_EQ(owner.allocations().device_allocations,allocation.device_allocations);
}

TEST_F(NodalOutput, LateAdvanceFailureCannotChangeVisibleFrameAndRetryCanPublish) {
    Case input; input.inverse={{4,4,4}};
    fea::FENodalState owner; ASSERT_EQ(input.Initialize(owner,1).status,fea::NodalStatus::Ok);
    visual::NodalMeshOutput output;
    ASSERT_EQ(output.Initialize(owner,Binding(owner)).status,visual::Status::Ok);
    ASSERT_EQ(output.Publish(owner).status,visual::Status::Ok);
    fea::NodalTrialToken token; fea::NodalAssemblyView view;
    ASSERT_EQ(owner.BeginTrial(&token,&view).status,fea::NodalStatus::Ok);
    FiniteForceCausingAdvanceOverflow<<<1,1,0,view.stream>>>(view);
    ASSERT_EQ(cudaGetLastError(),cudaSuccess);
    ASSERT_EQ(owner.SealAssembly(token).status,fea::NodalStatus::Ok);
    EXPECT_EQ(fea::AdvanceTranslations(owner,token).status,fea::NodalStatus::InvalidOutput);
    EXPECT_NE(owner.Commit(token).status,fea::NodalStatus::Ok);
    EXPECT_EQ(output.Publish(owner).status,visual::Status::StaleFrame);
    CheckMesh(output,0,0);
    ASSERT_EQ(owner.BeginTrial(&token,&view).status,fea::NodalStatus::Ok);
    ASSERT_EQ(owner.SealAssembly(token).status,fea::NodalStatus::Ok);
    ASSERT_EQ(fea::AdvanceTranslations(owner,token).status,fea::NodalStatus::Ok);
    ASSERT_EQ(owner.Commit(token).status,fea::NodalStatus::Ok);
    ASSERT_EQ(output.Publish(owner).status,visual::Status::Ok);
    CheckMesh(output,0,1);
    EXPECT_DOUBLE_EQ(output.surface().frame()->time,1);
}

TEST_F(NodalOutput, AnotherLiveSourceCannotReplaceTheBoundOwnersMesh) {
    Case input; fea::FENodalState first,second;
    ASSERT_EQ(input.Initialize(first).status,fea::NodalStatus::Ok);
    ASSERT_EQ(input.Initialize(second).status,fea::NodalStatus::Ok);
    visual::NodalMeshOutput output;
    EXPECT_EQ(output.Publish(first).status,visual::Status::NotInitialized);
    EXPECT_EQ(output.Initialize(second,Binding(first)).status,visual::Status::WrongOwner);
    ASSERT_EQ(output.Initialize(first,Binding(first)).status,visual::Status::Ok);
    ASSERT_EQ(output.Publish(first).status,visual::Status::Ok);
    fea::NodalTrialToken token; fea::NodalAssemblyView view;
    ASSERT_EQ(second.BeginTrial(&token,&view).status,fea::NodalStatus::Ok);
    AddAcceleration<<<1,1,0,view.stream>>>(view,2);
    ASSERT_EQ(cudaGetLastError(),cudaSuccess);
    ASSERT_EQ(second.SealAssembly(token).status,fea::NodalStatus::Ok);
    ASSERT_EQ(fea::AdvanceTranslations(second,token).status,fea::NodalStatus::Ok);
    ASSERT_EQ(second.Commit(token).status,fea::NodalStatus::Ok);
    EXPECT_GT(second.accepted().epoch,first.accepted().epoch);
    EXPECT_EQ(output.Publish(second).status,visual::Status::WrongOwner);
    CheckMesh(output,0,0);
}
}  // namespace
