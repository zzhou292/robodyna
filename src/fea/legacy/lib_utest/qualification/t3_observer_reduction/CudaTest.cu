#include "Fixture.h"
#include <cuda_runtime.h>
namespace t3_observer_test {
void CheckDeviceTruth(Fixture&,const b::Control&,unsigned,bool);
namespace {
__global__ void Serial(b::Storage* s,fe::NodalPreparedView view,q::BatchDiagnostics identity,
    const fe::shell_batch_plasticity_detail::MixedDeviceStorage* mixed) {
  frozen::FinalizeCandidate(s,&s->slab[0],&s->slab[1],view,identity,mixed);
}
struct DeviceFixture {
  Fixture source;
  b::Storage* storage=nullptr;
  Inputs* input=nullptr;
  Fields* fields=nullptr;
  fe::ShellSectionLaw* roles=nullptr;
  explicit DeviceFixture(unsigned parents=Parents,unsigned epoch=1):source(parents,epoch) {
    EXPECT_EQ(cudaMallocManaged(reinterpret_cast<void**>(&storage),source.layout.bytes),cudaSuccess);
    EXPECT_EQ(cudaMallocManaged(reinterpret_cast<void**>(&input),sizeof(Inputs)),cudaSuccess);
    EXPECT_EQ(cudaMallocManaged(reinterpret_cast<void**>(&fields),sizeof(Fields)),cudaSuccess);
    EXPECT_EQ(cudaMallocManaged(reinterpret_cast<void**>(&roles),parents*sizeof(*roles)),cudaSuccess);
  }
  ~DeviceFixture() {cudaFree(roles);cudaFree(fields);cudaFree(input);cudaFree(storage);}
  void Upload() {
    std::memcpy(storage,source.arena.data(),source.layout.bytes);
    *storage=source.layout.Rebase(*source.host,storage);
    std::memcpy(roles,source.roles.data(),source.roles.size()*sizeof(*roles));
    *input=source.source.input;input->mixed.law=roles;
    *fields=source.source.fields;
  }
  b::Control Evaluate(bool serial,unsigned epoch=1,bool assembled=true,bool catalog=true) {
    const auto view=fields->View(*input,epoch);
    const auto* mixed=catalog?&input->mixed:nullptr;
    if(serial)Serial<<<1,1>>>(storage,view,Identity(epoch,assembled),mixed);
    else b::LaunchMappedObserverDiagnostics(storage,&storage->slab[0],&storage->slab[1],view,Identity(epoch,assembled),mixed);
    EXPECT_EQ(cudaGetLastError(),cudaSuccess);
    EXPECT_EQ(cudaDeviceSynchronize(),cudaSuccess);
    return storage->control;
  }
  void UnchangedMechanics() const {
    const auto parents=source.roles.size();
    // Same initialized byte packets copied before either algorithm; this is
    // immutability, not a padded comparison of independently produced values.
    EXPECT_EQ(std::memcmp(storage->model.element,source.host->model.element,parents*sizeof(q::T3BatchElement)),0);
    for(unsigned s=0;s<2;++s)
      EXPECT_EQ(std::memcmp(storage->slab[s].element,source.host->slab[s].element,parents*sizeof(q::ForceTrial)),0);
    EXPECT_EQ(std::memcmp(fields,&source.source.fields,sizeof(Fields)),0);
    EXPECT_EQ(std::memcmp(input->values,source.source.input.values,sizeof(input->values)),0);
  }
};
}
TEST(T3ObserverCuda,ActualFixedTreesIndependentTruthAndUnchangedCaches) {
  for(unsigned count:{1u,129u,263u,33001u}) {
    SCOPED_TRACE(count);DeviceFixture f(count);f.Upload();
    const auto serial=f.Evaluate(true),actual=f.Evaluate(false);
    ASSERT_EQ(actual.status,q::BatchStatus::Success);SameExceptSums(actual,serial);
    CheckDeviceTruth(f.source,actual,1,true);SameControl(f.Evaluate(false),actual);
    f.UnchangedMechanics();
  }
  DeviceFixture f;f.Upload();
  EXPECT_NE(Bits(f.Evaluate(false).diagnostics.internal_work[0]),Bits(f.Evaluate(true).diagnostics.internal_work[0]));
}
TEST(T3ObserverCuda,InitialLaterMaskAndNoAssemblyBranches) {
  for(unsigned epoch:{0u,2u})for(unsigned mask=0;mask<8;++mask)for(bool assembled:{false,true}) {
    DeviceFixture f(Parents,epoch);
    for(unsigned p=0;p<3;++p) {
      auto& r=f.source.host->slab[1].element[p];auto h=r.proposed_history.data();h.active=mask&(1u<<p)?0:1;
      ASSERT_EQ(q::PrepareFailurePrescribedHistory(f.source.host->model.element[p].reference,h,
          {(epoch+1)*1e-6,epoch+1},r.proposed_history),q::Status::kSuccess);
    }
    f.Upload();const auto serial=f.Evaluate(true,epoch,assembled),actual=f.Evaluate(false,epoch,assembled);
    ASSERT_EQ(actual.status,q::BatchStatus::Success);SameExceptSums(actual,serial);
    CheckDeviceTruth(f.source,actual,epoch,assembled);f.UnchangedMechanics();
  }
}
TEST(T3ObserverCuda,ExactErrorReplayPrefixOverflowAndRetry) {
  for(unsigned fault=0;fault<8;++fault) {
    SCOPED_TRACE(fault);DeviceFixture f(263);
    auto& s=f.source;
    if(fault==0) {s.host->candidate_status[260]=q::Status::kInvalidInput;s.host->slab[1].element[1].internal_force[0].x=INFINITY;}
    if(fault==1)s.source.fields.orientation[4*(Nodes-1)]=0;
    if(fault==2)s.source.fields.position[3*(Nodes-1)]=INFINITY;
    if(fault==3)s.host->model.element[260].reference.area=-1;
    if(fault==4)s.host->slab[1].element[260].diagnostics.unscaled_element_dt=-0.;
    if(fault==5 || fault==6) {
      const double x=fault==5?DBL_MAX*.375:DBL_MAX;
      s.host->slab[1].element[0].diagnostics.internal_work_increment[0]=x;
      s.host->slab[1].element[1].diagnostics.internal_work_increment[0]=x;
      s.host->slab[1].element[2].diagnostics.internal_work_increment[0]=-x;
    }
    f.Upload();const bool catalog=fault!=7;
    SameControl(f.Evaluate(false,1,true,catalog),f.Evaluate(true,1,true,catalog));f.UnchangedMechanics();
  }
  DeviceFixture f;f.Upload();f.fields->orientation[4*(Nodes-1)]=0;
  EXPECT_NE(f.Evaluate(false).status,q::BatchStatus::Success);
  f.fields->orientation[4*(Nodes-1)]=1;
  const auto actual=f.Evaluate(false);ASSERT_EQ(actual.status,q::BatchStatus::Success);
  SameExceptSums(actual,f.Evaluate(true));CheckDeviceTruth(f.source,actual,1,true);
  f.UnchangedMechanics();
}
TEST(T3ObserverCuda,UnusedVelocityAndNegativeDtRemainInOriginalDomain) {
  DeviceFixture fixture;
  fixture.source.source.fields.velocity[3*(Nodes-1)]=NAN;
  fixture.source.source.fields.omega[3*(Nodes-1)]=INFINITY;
  auto& accepted=fixture.source.host->slab[0].element[0];
  Remove(fixture.source.host->model.element[0].reference,accepted,true);
  fixture.Upload();
  const auto actual=fixture.Evaluate(false);
  ASSERT_EQ(actual.status,q::BatchStatus::Success);
  SameExceptSums(actual,fixture.Evaluate(true));
  CheckDeviceTruth(fixture.source,actual,1,true);
  fixture.UnchangedMechanics();
  fixture.storage->slab[1].element[0].diagnostics.unscaled_element_dt=-1;
  SameControl(fixture.Evaluate(false),fixture.Evaluate(true));
  EXPECT_EQ(fixture.Evaluate(false).status,q::BatchStatus::Success);
}
} // namespace t3_observer_test
