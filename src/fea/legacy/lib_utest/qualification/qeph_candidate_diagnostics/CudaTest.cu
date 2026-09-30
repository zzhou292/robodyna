#include "Fixture.h"
#include <cuda_runtime.h>

namespace qeph_diagnostics_test {
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
  DeviceFixture() {
    EXPECT_EQ(cudaMallocManaged(reinterpret_cast<void**>(&storage),source.layout.bytes),cudaSuccess);
    EXPECT_EQ(cudaMallocManaged(reinterpret_cast<void**>(&input),sizeof(Inputs)),cudaSuccess);
    EXPECT_EQ(cudaMallocManaged(reinterpret_cast<void**>(&fields),sizeof(Fields)),cudaSuccess);
  }
  ~DeviceFixture() {cudaFree(fields);cudaFree(input);cudaFree(storage);}
  void Upload() {
    std::memcpy(storage,source.arena.data(),source.layout.bytes);
    *storage=source.layout.Rebase(*source.host,storage);
    *input=source.input;input->mixed.law=input->law;
    *fields=source.fields;
  }
  void Run(unsigned epoch,bool serial,bool catalog=true,bool assembled=true) {
    const auto view=fields->View(*input,epoch);
    const auto* mixed=catalog?&input->mixed:nullptr;
    if(serial)Serial<<<1,1>>>(storage,view,Identity(epoch,assembled),mixed);
    else b::LaunchMappedCandidateDiagnostics(storage,&storage->slab[0],&storage->slab[1],view,Identity(epoch,assembled),mixed);
    ASSERT_EQ(cudaGetLastError(),cudaSuccess);
    ASSERT_EQ(cudaDeviceSynchronize(),cudaSuccess);
  }
  void Assemble(unsigned epoch) {
    b::LaunchMappedAssembly(storage,&storage->slab[0],input->View(epoch),input->Cin(),&input->mixed,epoch==0);
    ASSERT_EQ(cudaGetLastError(),cudaSuccess);
    ASSERT_EQ(cudaDeviceSynchronize(),cudaSuccess);
  }
};
void UnchangedCaches(const DeviceFixture& f) {
  for(unsigned slab=0;slab<2;++slab)for(unsigned p=0;p<Parents;++p) {
    const auto& actual=f.storage->slab[slab].element[p];
    const auto& original=f.source.host->slab[slab].element[p];
    // The exact same initialized packet was copied before either algorithm;
    // this is an unchanged-byte test, not comparison of independently padded values.
    EXPECT_EQ(std::memcmp(&actual,&original,sizeof(actual)),0);
  }
}
}
TEST(QephCandidateDiagnosticsCuda,AllMasksAndEpochsMatchFrozenSerialEveryFieldBit) {
  DeviceFixture actual,serial;
  for(unsigned epoch=0;epoch<3;++epoch)for(unsigned mask=0;mask<8;++mask)for(bool assembled:{false,true}) {
    SCOPED_TRACE(::testing::Message()<<epoch<<":"<<mask<<":"<<assembled);
    actual.source.Reset(epoch,mask);serial.source.Reset(epoch,mask);
    actual.Upload();serial.Upload();
    actual.Run(epoch,false,true,assembled);serial.Run(epoch,true,true,assembled);
    ASSERT_EQ(actual.storage->control.status,q::BatchStatus::Success);
    SameControl(actual.storage->control,serial.storage->control);
    UnchangedCaches(actual);UnchangedCaches(serial);
  }
}
TEST(QephCandidateDiagnosticsCuda,CompetingLateFailuresRetryAndAssemblyScratchReuse) {
  for(unsigned fault=0;fault<10;++fault) {
    SCOPED_TRACE(fault);DeviceFixture actual,serial;
    actual.source.Reset(1);serial.source.Reset(1);
    for(auto* source:{&actual.source,&serial.source}) {
      if(fault<8)Fault(*source,fault);
      if(fault==9)source->host->candidate_status[2]=q::Status::kInvalidInput;
    }
    actual.Upload();serial.Upload();
    actual.Run(1,false,fault<8);serial.Run(1,true,fault<8);
    ASSERT_NE(actual.storage->control.status,q::BatchStatus::Success);
    SameControl(actual.storage->control,serial.storage->control);
    UnchangedCaches(actual);
  }
  DeviceFixture reused,fresh;
  reused.source.Reset(1);fresh.source.Reset(1);reused.Upload();fresh.Upload();
  reused.fields->orientation[4*(Nodes-1)]=0;
  reused.Run(1,false);ASSERT_NE(reused.storage->control.status,q::BatchStatus::Success);
  reused.fields->orientation[4*(Nodes-1)]=1;
  reused.Run(1,false);fresh.Run(1,true);
  ASSERT_EQ(reused.storage->control.status,q::BatchStatus::Success);
  SameControl(reused.storage->control,fresh.storage->control);
  // Candidate scratch is dead when the next accepted assembly begins. Its
  // displacement/quaternion/validation fields must never leak into that fold.
  fresh.Upload();reused.Assemble(1);fresh.Assemble(1);
  ASSERT_EQ(reused.storage->control.status,q::BatchStatus::Success);
  SameControl(reused.storage->control,fresh.storage->control);
  Compare(*reused.input,*fresh.input);
}
} // namespace qeph_diagnostics_test
