// Actual force-driven TL state -> Chrono mesh integration. Three free physical
// nodes carry explicit masses; no shell element, contact or Chrono dynamics.
#include "NodalMeshOutput.h"
#include "lib_src/solvers/ExplicitTranslationStep.h"
#include "lib_src/solvers/ExplicitNodalStep.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"
#include <gtest/gtest.h>
#include <array>
#include <cmath>
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
__global__ void CoupleCausingRotationLimit(fea::NodalAssemblyView view) {
    view.forces.couple_z[2]=1e6;
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

TEST_F(NodalOutput, StaggeredOwnerCannotEnterCollocatedOutputEvenAtEpochZero) {
    Case input; fea::FENodalState staggered,legacy;
    const double q[]{1,0,0,0,1,0,0,0,1,0,0,0},inverse_inertia[]{.25,.25,.25};
    fea::NodalStateConfig config; config.node_count=3;
    config.temporal_scheme=fea::NodalTemporalScheme::StaggeredHalfKickStart;
    ASSERT_EQ(staggered.Initialize(config,{input.x.data(),input.v.data(),nullptr,3,q},input.inverse.data(),
        fea::NodalDofConfig{input.fixed.data(),input.fixed.data(),inverse_inertia}).status,fea::NodalStatus::Ok);
    EXPECT_EQ(staggered.accepted().velocity_phase,fea::NodalVelocityPhase::Collocated);
    visual::NodalMeshOutput output;
    EXPECT_EQ(output.Initialize(staggered,Binding(staggered)).status,visual::Status::InvalidBinding);
    EXPECT_EQ(output.surface().frame(),nullptr);
    fea::NodalTrialToken token; fea::NodalAssemblyView view;
    ASSERT_EQ(staggered.BeginTrial(&token,&view).status,fea::NodalStatus::Ok);
    ASSERT_EQ(staggered.SealAssembly(token).status,fea::NodalStatus::Ok);
    ASSERT_EQ(fea::AdvanceStaggeredPrescribed(staggered,token,
        {view.owner_id,view.accepted.base_epoch,view.attempt,config.fixed_dt,1}).status,fea::NodalStatus::Ok);
    ASSERT_EQ(staggered.Commit(token).status,fea::NodalStatus::Ok);
    EXPECT_EQ(staggered.accepted().velocity_phase,fea::NodalVelocityPhase::PreviousMidpoint);
    EXPECT_EQ(output.Initialize(staggered,Binding(staggered)).status,visual::Status::InvalidBinding);
    ASSERT_EQ(input.Initialize(legacy).status,fea::NodalStatus::Ok);
    ASSERT_EQ(output.Initialize(legacy,Binding(legacy)).status,visual::Status::Ok);
    ASSERT_EQ(output.Publish(legacy).status,visual::Status::Ok); CheckMesh(output,0,0);
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

TEST_F(NodalOutput, ExtendedRotationOwnerReusesAcceptedMeshBridgeAndRejectsFailedSpin) {
    Case input;
    std::array<double,12> q{{1,0,0,0, 1,0,0,0, 1,0,0,0}};
    std::array<double,9> omega{{0,0,.2, 0,0,.2, 0,0,.2}};
    std::array<double,3> inverse_inertia{{1,1,1}};
    for(unsigned i=0;i<3;++i) input.v[3*i]=.2;
    fea::FENodalState owner; fea::NodalStateConfig config; config.node_count=3; config.fixed_dt=.01;
    ASSERT_EQ(owner.Initialize(config,{input.x.data(),input.v.data(),omega.data(),3,q.data()},input.inverse.data(),
                              fea::NodalDofConfig{input.fixed.data(),input.fixed.data(),inverse_inertia.data()}).status,
              fea::NodalStatus::Ok);
    visual::NodalMeshOutput output;
    ASSERT_EQ(output.Initialize(owner,Binding(owner)).status,visual::Status::Ok);
    ASSERT_EQ(output.Publish(owner).status,visual::Status::Ok);
    for(unsigned attempt=0;attempt<3;++attempt) {
        fea::NodalTrialToken token; fea::NodalAssemblyView view;
        ASSERT_EQ(owner.BeginTrial(&token,&view).status,fea::NodalStatus::Ok);
        if(attempt==1) CoupleCausingRotationLimit<<<1,1,0,view.stream>>>(view);
        ASSERT_EQ(owner.SealAssembly(token).status,fea::NodalStatus::Ok);
        const fea::NodalStepAdmission admission{view.owner_id,view.accepted.base_epoch,view.attempt,.01,.1,
                                               fea::NodalStepAdmissionKind::PrescribedConstantLoads};
        const auto advance=fea::AdvanceNodal(owner,token,admission);
        if(attempt==1) {
            EXPECT_EQ(advance.status,fea::NodalStatus::StepTooLarge);
            EXPECT_NE(owner.Commit(token).status,fea::NodalStatus::Ok);
            EXPECT_EQ(output.Publish(owner).status,visual::Status::StaleFrame);
            CheckMesh(output,.002,1);
        } else {
            ASSERT_EQ(advance.status,fea::NodalStatus::Ok);
            EXPECT_EQ(output.Publish(owner).status,visual::Status::StaleFrame);
            ASSERT_EQ(owner.Commit(token).status,fea::NodalStatus::Ok);
            ASSERT_EQ(output.Publish(owner).status,visual::Status::Ok);
            CheckMesh(output,attempt==0?.002:.004,attempt==0?1:2);
        }
    }
    // Geometry remains sourced from accepted translations. Nodal rotations
    // are not invented visual deformation or a second dynamics clock.
    EXPECT_NEAR(output.surface().frame()->time,.02,1e-16);
}

// These output tests use known prescribed loads, not coupled shell dynamics.
struct StaggeredCase : Case {
    std::array<double,12> q{{1,0,0,0, 1,0,0,0, 1,0,0,0}};
    std::array<double,9> omega{{0,0,.2, 0,0,.2, 0,0,.2}};
    std::array<double,3> inverse_inertia{{1,1,1}};
    fea::NodalReport Initialize(fea::FENodalState& owner, double h = .01) const {
        fea::NodalStateConfig config; config.node_count=3; config.fixed_dt=h;
        config.temporal_scheme=fea::NodalTemporalScheme::StaggeredHalfKickStart;
        return owner.Initialize(config,{x.data(),v.data(),omega.data(),3,q.data()},inverse.data(),
            fea::NodalDofConfig{fixed.data(),fixed.data(),inverse_inertia.data()});
    }
};

void SameStamp(const fea::NodalStamp& a, const fea::NodalStamp& b) {
    EXPECT_EQ(a.owner_id,b.owner_id); EXPECT_EQ(a.epoch,b.epoch); EXPECT_EQ(a.node_count,b.node_count);
    EXPECT_DOUBLE_EQ(a.time,b.time); EXPECT_DOUBLE_EQ(a.fixed_dt,b.fixed_dt);
    EXPECT_EQ(a.has_rotations,b.has_rotations); EXPECT_EQ(a.reactions_valid,b.reactions_valid);
    EXPECT_EQ(a.reaction_base_epoch,b.reaction_base_epoch); EXPECT_DOUBLE_EQ(a.reaction_time,b.reaction_time);
    EXPECT_EQ(a.temporal_scheme,b.temporal_scheme); EXPECT_EQ(a.velocity_phase,b.velocity_phase);
    EXPECT_DOUBLE_EQ(a.velocity_time,b.velocity_time); EXPECT_DOUBLE_EQ(a.reaction_kick_dt,b.reaction_kick_dt);
}

void CheckRawCapture(fea::FENodalState& owner, const visual::NodalMeshOutput& output) {
    std::array<double,9> x{},v{},omega{}; std::array<double,12> q{}; fea::NodalStamp stamp;
    ASSERT_EQ(owner.CopyAccepted({x.data(),v.data(),3,q.data(),omega.data()},&stamp).status,fea::NodalStatus::Ok);
    ASSERT_NE(output.stamp(),nullptr);
    SameStamp(*output.stamp(),stamp);
    const auto fields=output.fields(); ASSERT_EQ(fields.node_count,3);
    ASSERT_NE(fields.position_xyz,nullptr); ASSERT_NE(fields.velocity_xyz,nullptr);
    ASSERT_NE(fields.orientation_wxyz,nullptr); ASSERT_NE(fields.angular_velocity_xyz,nullptr);
    for(unsigned i=0;i<9;++i) {
        EXPECT_DOUBLE_EQ(fields.position_xyz[i],x[i]); EXPECT_DOUBLE_EQ(fields.velocity_xyz[i],v[i]);
        EXPECT_DOUBLE_EQ(fields.angular_velocity_xyz[i],omega[i]);
    }
    for(unsigned i=0;i<12;++i) EXPECT_DOUBLE_EQ(fields.orientation_wxyz[i],q[i]);
    EXPECT_EQ(output.surface().frame()->epoch,stamp.epoch);
    EXPECT_DOUBLE_EQ(output.surface().frame()->time,stamp.time);
}

TEST_F(NodalOutput, ExplicitStaggeredCaptureRetainsActualEndpointAndMidpointFields) {
    StaggeredCase input; for(unsigned i=0;i<3;++i) input.v[3*i]=.2;
    fea::FENodalState owner,legacy;
    ASSERT_EQ(input.Initialize(owner).status,fea::NodalStatus::Ok);
    const Case legacy_input; ASSERT_EQ(legacy_input.Initialize(legacy).status,fea::NodalStatus::Ok);
    visual::NodalMeshOutput output;
    EXPECT_EQ(output.Initialize(owner,Binding(owner),static_cast<visual::NodalOutputTiming>(99)).status,
              visual::Status::InvalidBinding);
    EXPECT_EQ(output.Initialize(legacy,Binding(legacy),visual::NodalOutputTiming::StaggeredHalfKick).status,
              visual::Status::InvalidBinding);
    EXPECT_EQ(output.surface().binding(),nullptr); EXPECT_EQ(output.stamp(),nullptr);
    EXPECT_EQ(output.fields().node_count,0); EXPECT_EQ(output.fields().orientation_wxyz,nullptr);
    ASSERT_EQ(output.Initialize(owner,Binding(owner),visual::NodalOutputTiming::StaggeredHalfKick).status,
              visual::Status::Ok);
    EXPECT_EQ(output.stamp(),nullptr); EXPECT_EQ(output.fields().position_xyz,nullptr);
    ASSERT_EQ(output.Publish(owner).status,visual::Status::Ok);
    ASSERT_NO_FATAL_FAILURE(CheckRawCapture(owner,output));
    EXPECT_EQ(output.stamp()->velocity_phase,fea::NodalVelocityPhase::Collocated);
    EXPECT_DOUBLE_EQ(output.stamp()->time,0); EXPECT_DOUBLE_EQ(output.stamp()->velocity_time,0);
    EXPECT_DOUBLE_EQ(output.stamp()->reaction_kick_dt,0);
    const auto allocations=owner.allocations();
    for(std::uint64_t n=1;n<=5;++n) {
        fea::NodalTrialToken token; fea::NodalAssemblyView view;
        ASSERT_EQ(owner.BeginTrial(&token,&view).status,fea::NodalStatus::Ok);
        AddAcceleration<<<1,1,0,view.stream>>>(view,2);
        ASSERT_EQ(cudaGetLastError(),cudaSuccess);
        ASSERT_EQ(owner.SealAssembly(token).status,fea::NodalStatus::Ok);
        ASSERT_EQ(fea::AdvanceStaggeredPrescribed(owner,token,
            {view.owner_id,view.accepted.base_epoch,view.attempt,.01,1}).status,fea::NodalStatus::Ok);
        EXPECT_EQ(output.Publish(owner).status,visual::Status::StaleFrame);
        EXPECT_EQ(output.stamp()->epoch,n-1);
        ASSERT_EQ(owner.Commit(token).status,fea::NodalStatus::Ok);
        ASSERT_EQ(output.Publish(owner).status,visual::Status::Ok);
        ASSERT_NO_FATAL_FAILURE(CheckRawCapture(owner,output));
        const auto& stamp=*output.stamp();
        EXPECT_EQ(stamp.velocity_phase,fea::NodalVelocityPhase::PreviousMidpoint);
        EXPECT_EQ(stamp.reaction_base_epoch,n-1);
        EXPECT_DOUBLE_EQ(stamp.velocity_time,stamp.reaction_time+.005);
        EXPECT_DOUBLE_EQ(stamp.reaction_kick_dt,n==1?.005:.01);
        EXPECT_LT(stamp.velocity_time,stamp.time);
        for(unsigned i=0;i<3;++i) {
            // Actual raw v is at the midpoint. Endpoint reconstruction would
            // add another .01 m/s and must not enter this capture.
            EXPECT_NEAR(output.fields().velocity_xyz[3*i],.2+2*.01*(n-.5),2e-15);
            EXPECT_NEAR(output.fields().position_xyz[3*i],.2*n*.01+n*n*.0001,2e-15);
            EXPECT_NEAR(output.fields().orientation_wxyz[4*i],std::cos(.1*n*.01),2e-15);
            EXPECT_NEAR(output.fields().orientation_wxyz[4*i+3],std::sin(.1*n*.01),2e-15);
        }
    }
    EXPECT_EQ(owner.allocations().device_allocations,allocations.device_allocations);
    EXPECT_EQ(owner.allocations().device_bytes,allocations.device_bytes);
    // The unchanged default remains usable without rotation storage.
    visual::NodalMeshOutput collocated;
    ASSERT_EQ(collocated.Initialize(legacy,Binding(legacy)).status,visual::Status::Ok);
    ASSERT_EQ(collocated.Publish(legacy).status,visual::Status::Ok);
    EXPECT_NE(collocated.fields().position_xyz,nullptr); EXPECT_NE(collocated.fields().velocity_xyz,nullptr);
    EXPECT_EQ(collocated.fields().orientation_wxyz,nullptr); EXPECT_EQ(collocated.fields().angular_velocity_xyz,nullptr);
    EXPECT_FALSE(collocated.stamp()->has_rotations);
    EXPECT_DOUBLE_EQ(collocated.stamp()->velocity_time,collocated.stamp()->time);
}

__global__ void RestoreSecondNode(fea::NodalAssemblyView view) {
    if(blockIdx.x || threadIdx.x) return;
    view.forces.force_y[1]=2/view.mass.inverse_mass[1];
}

TEST_F(NodalOutput, FailedMeshPublicationPreservesRawCaptureThenLaterAcceptedStateCanPublish) {
    StaggeredCase input; input.v[4]=-1;
    fea::FENodalState owner,other;
    ASSERT_EQ(input.Initialize(owner,1).status,fea::NodalStatus::Ok);
    ASSERT_EQ(input.Initialize(other,1).status,fea::NodalStatus::Ok);
    visual::NodalMeshOutput output;
    ASSERT_EQ(output.Initialize(owner,Binding(owner),visual::NodalOutputTiming::StaggeredHalfKick).status,
              visual::Status::Ok);
    ASSERT_EQ(output.Publish(owner).status,visual::Status::Ok);
    const auto saved_stamp=*output.stamp(); const auto saved_fields=output.fields();
    const auto saved_vertices=output.surface().mesh()->GetCoordsVertices();
    EXPECT_EQ(output.Publish(other).status,visual::Status::WrongOwner);
    for(unsigned step=1;step<=2;++step) {
        fea::NodalTrialToken token; fea::NodalAssemblyView view;
        ASSERT_EQ(owner.BeginTrial(&token,&view).status,fea::NodalStatus::Ok);
        if(step==2) RestoreSecondNode<<<1,1,0,view.stream>>>(view);
        ASSERT_EQ(cudaGetLastError(),cudaSuccess);
        ASSERT_EQ(owner.SealAssembly(token).status,fea::NodalStatus::Ok);
        ASSERT_EQ(fea::AdvanceStaggeredPrescribed(owner,token,
            {view.owner_id,view.accepted.base_epoch,view.attempt,1,1}).status,fea::NodalStatus::Ok);
        ASSERT_EQ(owner.Commit(token).status,fea::NodalStatus::Ok);
        if(step==1) {
            // A legitimate accepted owner state collapses the display triangle.
            // The complete readback has succeeded before surface rejection.
            EXPECT_EQ(output.Publish(owner).status,visual::Status::InvalidFrame);
            EXPECT_EQ(output.Publish(owner).status,visual::Status::InvalidFrame);
            SameStamp(*output.stamp(),saved_stamp);
            EXPECT_EQ(output.fields().position_xyz,saved_fields.position_xyz);
            EXPECT_EQ(output.fields().orientation_wxyz,saved_fields.orientation_wxyz);
            for(unsigned i=0;i<9;++i) {
                EXPECT_DOUBLE_EQ(output.fields().position_xyz[i],input.x[i]);
                EXPECT_DOUBLE_EQ(output.fields().velocity_xyz[i],input.v[i]);
                EXPECT_DOUBLE_EQ(output.fields().angular_velocity_xyz[i],input.omega[i]);
            }
            for(unsigned i=0;i<12;++i) EXPECT_DOUBLE_EQ(output.fields().orientation_wxyz[i],input.q[i]);
            EXPECT_EQ(output.surface().frame()->epoch,0);
            for(unsigned i=0;i<3;++i) {
                const auto& actual=output.surface().mesh()->GetCoordsVertices()[i];
                EXPECT_DOUBLE_EQ(actual.x(),saved_vertices[i].x());
                EXPECT_DOUBLE_EQ(actual.y(),saved_vertices[i].y());
                EXPECT_DOUBLE_EQ(actual.z(),saved_vertices[i].z());
            }
        } else {
            ASSERT_EQ(output.Publish(owner).status,visual::Status::Ok);
            ASSERT_NO_FATAL_FAILURE(CheckRawCapture(owner,output));
            EXPECT_EQ(output.stamp()->epoch,2); EXPECT_DOUBLE_EQ(output.stamp()->time,2);
            EXPECT_DOUBLE_EQ(output.stamp()->velocity_time,1.5);
            EXPECT_DOUBLE_EQ(output.fields().position_xyz[4],1);
        }
    }
}
}  // namespace
