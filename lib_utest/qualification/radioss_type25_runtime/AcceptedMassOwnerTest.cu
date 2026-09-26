// SPDX-License-Identifier: AGPL-3.0-or-later
#include <gtest/gtest.h>
#include "AcceptedMassRig.h"
#include "lib_src/collision/radioss_type25/runtime/MassOperands.h"
#include "../radioss_type25_friction/CudaFixture.h"
#include "../radioss_type25_friction/Assertions.h"
#include <limits>
#include <cstring>
namespace accepted_mass_runtime_test {
namespace q=n::runtime_qualification;namespace rd=n::runtime_detail;
void SameBits(const std::vector<double>& a,const std::vector<double>& b) {
  ASSERT_EQ(a.size(),b.size());EXPECT_EQ(std::memcmp(a.data(),b.data(),a.size()*sizeof(double)),0);
}
__global__ void ReadNativeMass(rd::MassOperands mass,n::units_detail::Factors units,double* output,unsigned* invalid,std::size_t n) {
  for(std::size_t i=threadIdx.x;i<n;i+=blockDim.x)if(!rd::NativeMass(mass,i,units,output[i]))atomicAdd(invalid,1u);
}
// Test-only owning readback: an authenticated accepted pointer is consumed by
// the same per-load conversion leaf as FOR3. No value is written into the owner.
std::vector<double> Converted(const fe::NodalAcceptedRawMassView& view,double mass_scale) {
  n::units_detail::Factors units;EXPECT_TRUE(n::units_detail::Make({1,mass_scale,1},units));
  double* values=nullptr;unsigned* invalid=nullptr;std::vector<double> out(view.node_count);unsigned rejected=1;
  const auto fail=[](cudaError_t e){if(e!=cudaSuccess)throw std::runtime_error(cudaGetErrorString(e));};
  try {
    fail(cudaMalloc(&values,out.size()*sizeof(double)));fail(cudaMalloc(&invalid,sizeof(unsigned)));
    fail(cudaMemsetAsync(invalid,0,sizeof(unsigned),view.stream));
    ReadNativeMass<<<1,32,0,view.stream>>>({view.mass_kg,rd::MassOperands::Units::Si},units,values,invalid,out.size());
    fail(cudaGetLastError());fail(cudaMemcpyAsync(out.data(),values,out.size()*sizeof(double),cudaMemcpyDeviceToHost,view.stream));
    fail(cudaMemcpyAsync(&rejected,invalid,sizeof(unsigned),cudaMemcpyDeviceToHost,view.stream));
    fail(cudaStreamSynchronize(view.stream));EXPECT_EQ(rejected,0u);
  } catch(...) {cudaStreamSynchronize(view.stream);if(values)cudaFree(values);if(invalid)cudaFree(invalid);throw;}
  fail(cudaFree(values));fail(cudaFree(invalid));return out;
}
TEST(NativeType25AcceptedMassCuda, EmptyCinOwnerMatchesStaticContactThroughActiveCommonCommits) {
  moving_cache_test::Rig legacy(true),current(true);
  current.config.response_mass=n::ResponseMassPolicy::AcceptedOwnerCoefficients;
  legacy.Initialize();current.Initialize();
  EXPECT_LE(current.contact.allocations().runtime_device_bytes,legacy.contact.allocations().runtime_device_bytes);
  EXPECT_LT(current.contact.allocations().startup_host_bytes,legacy.contact.allocations().startup_host_bytes);
  std::size_t active=0;
  for(unsigned step=0;step<4;++step) {
    SCOPED_TRACE(step);
    Attempt a,b;legacy.Begin(a);current.Begin(b);
    Check(legacy.contact.AssembleAccepted(legacy.owner,a.token,a.assembly));
    Check(current.contact.AssembleAccepted(current.owner,b.token,b.assembly));
    q::Observation left,right;
    ASSERT_TRUE(q::Access::Read(legacy.contact,legacy.owner,a.token,a.assembly,&left));
    ASSERT_TRUE(q::Access::Read(current.contact,current.owner,b.token,b.assembly,&right));
    ASSERT_EQ(left.response.size(),right.response.size());
    for(std::size_t i=0;i<left.response.size();++i)type25_friction_test::Same(left.response[i],right.response[i],true);
    EXPECT_EQ(left.cohort_ends,right.cohort_ends);active+=current.contact.last_diagnostics().active_forces;
    legacy.Prepare(a);current.Prepare(b);Check(legacy.Commit(a));Check(current.Commit(b));
    SameBits(legacy.Positions(),current.Positions());
  }
  EXPECT_GT(active,0u);
}
TEST(NativeType25AcceptedMassCuda, RealCinMassTransferAndRollbackUseTheAcceptedSlab) {
  CinRig rig;rig.Initialize();const auto original=rig.Raw();const auto allocations=rig.contact.allocations();
  for(unsigned step=0;step<3;++step) {
    SCOPED_TRACE(step);
    const auto before=rig.Raw();const auto positions=rig.Positions();
    Attempt a;rig.Begin(a);fe::NodalAcceptedRawMassView view;
    Check(rig.owner.BorrowAcceptedRawMass(a.token,&view));
    const auto native=Converted(view,1000.);
    for(std::size_t i=0;i<native.size();++i)EXPECT_EQ(native[i],before[i]/1000.);
    if(step) {EXPECT_EQ(native[4],0.);EXPECT_GT(native[0],original[0]/1000.);}
    Check(rig.contact.AssembleAccepted(rig.owner,a.token,a.assembly));
    EXPECT_EQ(rig.contact.accepted().stamp.epoch,step);
    rig.Prepare(a);SameBits(rig.Raw(),before);
    if(step==1) {
      const auto accepted=rig.contact.accepted();rig.Discard();
      SameBits(rig.Raw(),before);SameBits(rig.Positions(),positions);
      EXPECT_EQ(rig.contact.accepted().generation,accepted.generation);
      EXPECT_EQ(rig.contact.accepted().selectors.history,accepted.selectors.history);
      Attempt retry;rig.Begin(retry);
      EXPECT_NE(rig.owner.AuthenticateAcceptedRawMass(retry.token,view).status,fe::NodalStatus::Ok);
      Check(rig.contact.AssembleAccepted(rig.owner,retry.token,retry.assembly));rig.Prepare(retry);Check(rig.Commit(retry));
    } else Check(rig.Commit(a));
    const auto after=rig.Raw();EXPECT_EQ(after[4],0.);EXPECT_GT(after[0],original[0]);
    EXPECT_EQ(rig.contact.accepted().stamp.epoch,step+1);EXPECT_EQ(rig.contact.accepted().force_base_stamp.epoch,step);
    EXPECT_EQ(rig.contact.allocations().device_bytes,allocations.device_bytes);
  }
}
TEST(NativeType25AcceptedMassCuda, StaticPolicyStillRejectsRealCinAndMissingWitnessCannotCommit) {
  {
    CinRig rig;rig.config.response_mass=n::ResponseMassPolicy::StaticPhysicalLedger;rig.Initialize();
    const auto before=rig.Raw();Attempt a;rig.Begin(a);
    EXPECT_EQ(rig.contact.AssembleAccepted(rig.owner,a.token,a.assembly).status,n::TransactionStatus::UnsupportedProfile);
    SameBits(rig.Raw(),before);EXPECT_EQ(rig.contact.accepted().generation,0u);
  }
  {
    CinRig rig;rig.Initialize();const auto before=rig.Raw();Attempt a;
    Check(rig.owner.BeginTrial(&a.token,&a.assembly));
    Check(rig.triangle.AssembleMappedAccepted(rig.owner,a.token,a.assembly)); // Actual Q4 witness absent.
    Check(rig.contact.AssembleAccepted(rig.owner,a.token,a.assembly));
    fe::NodalCinAssemblyView cin_view;Check(rig.owner.BorrowCinAssembly(a.token,&cin_view));
    std::uint8_t flag=99;type25_friction_test::Drain drain{a.assembly.stream};
    ASSERT_EQ(cudaMemcpyAsync(&flag,cin_view.witness_activity,1,cudaMemcpyDeviceToHost,a.assembly.stream),cudaSuccess);
    ASSERT_EQ(cudaStreamSynchronize(a.assembly.stream),cudaSuccess);EXPECT_EQ(flag,0);
    Check(rig.owner.SealAssembly(a.token));
    EXPECT_NE(fe::AdvanceStaggeredCin(rig.owner,a.token,{a.assembly.owner_id,a.assembly.accepted.base_epoch,a.assembly.attempt,
        rig.source.physical.Qualification,rig.source.physical.fixed_dt,.2,true}).status,fe::NodalStatus::Ok);
    rig.Discard();SameBits(rig.Raw(),before);EXPECT_EQ(rig.contact.accepted().generation,0u);
    Attempt retry;rig.Begin(retry);Check(rig.contact.AssembleAccepted(rig.owner,retry.token,retry.assembly));
    rig.Prepare(retry);Check(rig.Commit(retry));EXPECT_EQ(rig.owner.accepted().epoch,1u);
  }
}
TEST(NativeType25AcceptedMassCuda, RigidPhysicalMembersKeepRawMassAndLegacyPolicyStillRejectsGroups) {
  {
    RigidRig legacy;legacy.config.response_mass=n::ResponseMassPolicy::StaticPhysicalLedger;
    EXPECT_EQ(legacy.Initialize().status,n::TransactionStatus::UnsupportedProfile);
    EXPECT_FALSE(legacy.contact.accepted().available);
  }
  RigidRig rig;Check(rig.Initialize());ASSERT_EQ(rig.owner.accepted().rigid_groups.group_count,1u);
  std::size_t active=0;
  for(unsigned step=0;step<3;++step) {
    SCOPED_TRACE(step);
    Attempt a;rig.Begin(a);fe::NodalAcceptedRawMassView view;Check(rig.owner.BorrowAcceptedRawMass(a.token,&view));
    const auto mass=Converted(view,1.);
    for(const auto& member:rig.rigid.members())EXPECT_EQ(mass[member.domain_node],member.mass_kg);
    Check(rig.contact.AssembleAccepted(rig.owner,a.token,a.assembly));active+=rig.contact.last_diagnostics().active_forces;
    rig.Prepare(a);Check(rig.Commit(a));EXPECT_EQ(rig.contact.accepted().force_base_stamp.epoch,step);
  }
  EXPECT_GT(active,0u);
}
using ConversionCuda=type25_friction_test::PacketCuda<>;
TEST_F(ConversionCuda, DeviceConversionPreservesInvalidOutputsAndSignedZero) {
  const std::array<double,8> values{0.,-0.,.25,1000.,-1.,std::numeric_limits<double>::denorm_min(),
    std::numeric_limits<double>::infinity(),std::numeric_limits<double>::quiet_NaN()};
  std::array<double,8> result;result.fill(97.);unsigned rejected=0;
  type25_friction_test::Drain drain{stream};
  auto* flag=reinterpret_cast<unsigned*>(static_cast<double*>(output)+result.size());
  ASSERT_EQ(cudaMemcpyAsync(input,values.data(),sizeof(values),cudaMemcpyHostToDevice,stream),cudaSuccess);
  ASSERT_EQ(cudaMemcpyAsync(output,result.data(),sizeof(result),cudaMemcpyHostToDevice,stream),cudaSuccess);
  ASSERT_EQ(cudaMemsetAsync(flag,0,sizeof(unsigned),stream),cudaSuccess);
  n::units_detail::Factors units;ASSERT_TRUE(n::units_detail::Make({1,1000,1},units));
  ReadNativeMass<<<1,7,0,stream>>>({static_cast<const double*>(input),rd::MassOperands::Units::Si},units,
    static_cast<double*>(output),flag,result.size());
  ASSERT_EQ(cudaGetLastError(),cudaSuccess);
  ASSERT_EQ(cudaMemcpyAsync(result.data(),output,sizeof(result),cudaMemcpyDeviceToHost,stream),cudaSuccess);
  ASSERT_EQ(cudaMemcpyAsync(&rejected,flag,sizeof(rejected),cudaMemcpyDeviceToHost,stream),cudaSuccess);
  ASSERT_EQ(cudaStreamSynchronize(stream),cudaSuccess);EXPECT_EQ(rejected,4u);
  for(unsigned i=0;i<4;++i) {const double expected=values[i]/1000.;EXPECT_EQ(std::memcmp(&result[i],&expected,sizeof(double)),0);}
  for(unsigned i=4;i<8;++i)EXPECT_EQ(result[i],97.);
}
} // namespace accepted_mass_runtime_test
