#include "T3ForcePortCases.h"
#include <cuda_runtime.h>

namespace t3_force_port_test {
namespace {
struct Packet {
  port::ReferenceData reference;
  port::HistoryValues values;
  port::HistoryStamp stamp;
  port::History base;
  port::PrescribedInterval interval;
  port::ForceTrial result;
  port::Status status=port::Status::kInvalidInput;
};
static_assert(sizeof(Packet)<=8192,"One reusable T3 force qualification packet");
enum class Operation { PrepareHistory,Force,AliasedForce };
__global__ void Evaluate(Packet* p,Operation operation) {
  if(operation==Operation::PrepareHistory)
    p->status=port::PreparePrescribedHistory(p->reference,p->values,p->stamp,p->base);
  else if(operation==Operation::AliasedForce)
    p->status=port::EvaluateForce(p->reference,p->result.proposed_history,p->interval,p->result);
  else p->status=port::EvaluateForce(p->reference,p->base,p->interval,p->result);
}
class T3ForcePortCuda : public ::testing::Test {
 protected:
  Packet* device=nullptr;
  void SetUp() override {
    int count=0; ASSERT_EQ(cudaGetDeviceCount(&count),cudaSuccess); ASSERT_GT(count,0);
    ASSERT_EQ(cudaGetLastError(),cudaSuccess); ASSERT_EQ(cudaMalloc(&device,sizeof(Packet)),cudaSuccess);
  }
  void TearDown() override { if(device) EXPECT_EQ(cudaFree(device),cudaSuccess); }
  void Execute(Packet& p,Operation operation=Operation::Force) {
    ASSERT_EQ(cudaMemcpy(device,&p,sizeof(p),cudaMemcpyHostToDevice),cudaSuccess);
    Evaluate<<<1,1>>>(device,operation);
    ASSERT_EQ(cudaGetLastError(),cudaSuccess); ASSERT_EQ(cudaDeviceSynchronize(),cudaSuccess);
    ASSERT_EQ(cudaMemcpy(&p,device,sizeof(p),cudaMemcpyDeviceToHost),cudaSuccess);
  }
  void Prepare(Packet& p) {
    ASSERT_NO_FATAL_FAILURE(Execute(p,Operation::PrepareHistory)); ASSERT_EQ(p.status,port::Status::kSuccess);
    ASSERT_TRUE(p.base.matches_reference(p.reference));
  }
};
}
TEST_F(T3ForcePortCuda, CompleteNativeParityUsesActualDeviceHistoryAndForceOperations) {
  ForEachParityCase([&](const auto& input,const auto& values,const auto& in) {
    Packet p; p.reference=Reference(input); p.values=values; p.interval=in;
    ASSERT_NO_FATAL_FAILURE(Prepare(p));
    const auto reference_bytes=Bytes(p.reference); const auto history_bytes=Bytes(p.base); const auto interval_bytes=Bytes(p.interval);
    ASSERT_NO_FATAL_FAILURE(Execute(p)); ASSERT_EQ(p.status,port::Status::kSuccess);
    Check(p.reference,p.base,p.interval,p.result);
    EXPECT_EQ(Bytes(p.reference),reference_bytes); EXPECT_EQ(Bytes(p.base),history_bytes); EXPECT_EQ(Bytes(p.interval),interval_bytes);
  });
  RecordProperty("prescribed_native_configurations",ParityCases);
  RecordProperty("owned_device_bytes",static_cast<int>(sizeof(Packet)));
  RecordProperty("device_allocations",1); RecordProperty("threads_per_block",1);
}
TEST_F(T3ForcePortCuda, IndependentModesPhysicalPowerAndAcceptedByValueHistoryStayCoherent) {
  for(double scale:{1.,.02}) for(unsigned shape:{0u,1u}) for(unsigned mode=0;mode<8;++mode) for(double sign:{-1.,1.}) {
    SCOPED_TRACE(mode);
    Packet p; p.reference=Reference(Triangle(scale,shape)); p.values.thickness=p.reference.input.thickness;
    p.interval=Interval(p.reference.input,1e-4); Mode(p.interval,mode,sign*.001);
    ASSERT_NO_FATAL_FAILURE(Prepare(p));
    ASSERT_NO_FATAL_FAILURE(Execute(p)); ASSERT_EQ(p.status,port::Status::kSuccess);
    Check(p.reference,p.base,p.interval,p.result,true); Power(p.reference,p.interval,p.result); Balance(p.interval,p.result);
  }
  Packet p; p.reference=Reference(Triangle(.02)); p.values=Values(oracle::Seed(p.reference.input.thickness));
  ASSERT_NO_FATAL_FAILURE(Prepare(p));
  for(double rate:{.001,0.,-.001,0.}) {
    p.interval=Interval(p.reference.input,1e-4); p.interval.base_time=p.base.stamp().time;
    p.interval.sample_index=p.base.stamp().sample_index+1; Mode(p.interval,0,rate);
    ASSERT_NO_FATAL_FAILURE(Execute(p)); ASSERT_EQ(p.status,port::Status::kSuccess);
    Check(p.reference,p.base,p.interval,p.result,true);
    if(rate==0) {
      for(unsigned i=0;i<5;++i) EXPECT_EQ(p.result.proposed_history.data().stress[i],p.result.proposed_history.data().material_stress[i]);
      for(unsigned i=0;i<2;++i) EXPECT_EQ(p.result.proposed_history.data().internal_work[i],p.base.data().internal_work[i]);
      EXPECT_EQ(p.result.proposed_history.data().equivalent_strain_rate,0.);
    }
    const auto result=p.result;
    ASSERT_NO_FATAL_FAILURE(Execute(p)); ASSERT_EQ(p.status,port::Status::kSuccess); Exact(p.result,result);
    p.base=p.result.proposed_history;
  }
  // Independently check a changed current area without rebuilding reference.
  p.interval=Interval(p.reference.input,1e-4); p.interval.base_time=p.base.stamp().time;
  p.interval.sample_index=p.base.stamp().sample_index+1;
  p.interval.position[1].x*=1.25; p.interval.position[2].y*=.75; p.interval.position[2].z=.003;
  Mode(p.interval,2,.001);
  ASSERT_NO_FATAL_FAILURE(Execute(p)); ASSERT_EQ(p.status,port::Status::kSuccess);
  Check(p.reference,p.base,p.interval,p.result,true); Power(p.reference,p.interval,p.result); Balance(p.interval,p.result);
  EXPECT_NE(p.result.kinematics.area,p.reference.area);
}
TEST_F(T3ForcePortCuda, EveryHistorySlotLateFailuresAndAliasedRetryPreserveDeviceOutputs) {
  Packet p; p.reference=Reference(Triangle(.02)); p.values.thickness=p.reference.input.thickness;
  p.interval=Interval(p.reference.input,1e-4); Mode(p.interval,0,.001);
  ASSERT_NO_FATAL_FAILURE(Prepare(p));
  ASSERT_NO_FATAL_FAILURE(Execute(p)); ASSERT_EQ(p.status,port::Status::kSuccess);
  const auto valid=p; const auto clean=p.result;
  for(unsigned fault=0;fault<InvalidForceCases;++fault) {
    SCOPED_TRACE(fault);
    p=valid; Fault(fault,p.reference,p.base,p.interval);
    const auto rb=Bytes(p.reference); const auto hb=Bytes(p.base); const auto ib=Bytes(p.interval);
    const auto output_bytes=Bytes(p.result);
    ASSERT_NO_FATAL_FAILURE(Execute(p)); EXPECT_NE(p.status,port::Status::kSuccess);
    EXPECT_EQ(Bytes(p.result),output_bytes); EXPECT_EQ(Bytes(p.reference),rb); EXPECT_EQ(Bytes(p.base),hb); EXPECT_EQ(Bytes(p.interval),ib);
    if(fault==9||fault==10) EXPECT_EQ(p.status,port::Status::kNonfiniteResult);
  }
  std::array<double,26> fields{}; native::detail::PackHistory(Native(valid.base.data()),fields);
  for(unsigned field=0;field<26;++field) {
    SCOPED_TRACE(field);
    p=valid; auto invalid=fields; invalid[field]=std::numeric_limits<double>::quiet_NaN();
    p.values=Values(native::detail::UnpackHistory(invalid.data()));
    const auto history_bytes=Bytes(p.base);
    const auto output_bytes=Bytes(p.result);
    ASSERT_NO_FATAL_FAILURE(Execute(p,Operation::PrepareHistory)); EXPECT_EQ(p.status,port::Status::kInvalidInput);
    EXPECT_EQ(Bytes(p.base),history_bytes); EXPECT_EQ(Bytes(p.result),output_bytes);
  }
  for(unsigned field:{21u,24u,25u}) {
    p=valid; auto invalid=fields; invalid[field]=-1; p.values=Values(native::detail::UnpackHistory(invalid.data()));
    const auto history_bytes=Bytes(p.base);
    ASSERT_NO_FATAL_FAILURE(Execute(p,Operation::PrepareHistory)); EXPECT_EQ(p.status,port::Status::kInvalidInput);
    EXPECT_EQ(Bytes(p.base),history_bytes);
  }
  p=valid; ASSERT_NO_FATAL_FAILURE(Execute(p)); ASSERT_EQ(p.status,port::Status::kSuccess); Exact(p.result,clean);
  p.interval.base_time=p.result.proposed_history.stamp().time; p.interval.sample_index=2;
  const auto accepted_history=p.result.proposed_history; const auto next_interval=p.interval;
  Mode(p.interval,0,1e6); const auto before_alias=Bytes(p.result);
  ASSERT_NO_FATAL_FAILURE(Execute(p,Operation::AliasedForce)); EXPECT_EQ(p.status,port::Status::kNonfiniteResult);
  EXPECT_EQ(Bytes(p.result),before_alias);
  p.interval=next_interval;
  Packet separate=p; separate.base=accepted_history;
  ASSERT_NO_FATAL_FAILURE(Execute(separate)); ASSERT_EQ(separate.status,port::Status::kSuccess);
  ASSERT_NO_FATAL_FAILURE(Execute(p,Operation::AliasedForce)); ASSERT_EQ(p.status,port::Status::kSuccess);
  Exact(p.result,separate.result); Check(p.reference,accepted_history,p.interval,p.result,true);
}
} // namespace t3_force_port_test
