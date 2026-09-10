#pragma once
#include "case/source_assembly_dynamics/State.h"
#include "case/source_assembly/tests/SourceAssemblyBindingTestSupport.h"
#include "output/source_assembly/SourceAssemblyAcceptedOutput.h"
#include <cuda_runtime_api.h>
#include <gtest/gtest.h>

namespace crash::cases::source_assembly_dynamics {
struct SourceAssemblyDynamicsTestAccess {
    enum class Fault { LastWallFace,AppliedLoad };
    enum class ForceFault { LastNode,LastGroup,CaptureIdentity,GroupIdentity,CaptureAssociation };
    static observation::ForceStageInput CapturedInput(const SourceAssemblyWallCase&,const Sample& before);
    static std::array<std::uintptr_t,4> CaptureStorage(const SourceAssemblyWallCase&);
    static Report RejectForceStage(SourceAssemblyWallCase&,ForceFault);
    static Report RejectCaptureDevice(SourceAssemblyWallCase&);
    static Report RejectSpin(SourceAssemblyWallCase&,bool bad_phase=false);

    static const Sample& Accepted(const SourceAssemblyWallCase& c) { return c.impl_->accepted(); }
    static Report RejectLate(SourceAssemblyWallCase& c,Fault fault) {
        auto& s=*c.impl_;auto r=s.Prepare();if(!r)return s.Stop(r);
        r=s.Evaluate();if(!r)return s.Stop(r);
        if(fault==Fault::LastWallFace)s.candidate().wall.wall_face.back()=UINT64_MAX;
        else s.applied_force.back()+=1000;
        r=s.Check();
        if(r)return s.Stop(Failure(Status::ComponentFailure,"Injected late validation fault unexpectedly passed"));
        return s.Stop(r);
    }
};
namespace test {
namespace source=source_assembly;
using output::assembly::SourceAssemblyAcceptedOutput;
inline constexpr double Dt=1./67108864;
Config SmokeConfig();
source::SourceAssemblyWallSetup PrepareWall(const source::SourceAssemblyBindings&,double gap=5e-6);
class SourceAssemblyDynamicsCheck:public ::testing::Test {
  protected:
    void SetUp() override { int devices=0;ASSERT_EQ(cudaGetDeviceCount(&devices),cudaSuccess);ASSERT_GT(devices,0); }
};
void SameFields(const Fields&,const Fields&);
void SameParents(const ParentFields&,const ParentFields&);
void SameAccepted(const Sample&,const Sample&);
void SameAllocations(const SourceAssemblyWallCase&,fe::NodalAllocationInfo,std::size_t);
void CheckOutput(SourceAssemblyWallCase&,SourceAssemblyAcceptedOutput&);
} // namespace test
} // namespace crash::cases::source_assembly_dynamics
