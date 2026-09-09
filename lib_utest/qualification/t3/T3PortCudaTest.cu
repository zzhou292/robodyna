#include "T3PortFixture.h"
#include <cuda_runtime.h>
namespace t3_port_test {
namespace {
struct Packet {
  port::ReferenceInput input;
  port::ReferenceData reference;
  port::PrescribedInterval interval;
  port::Kinematics result;
  port::Status status=port::Status::kInvalidInput;
};
static_assert(sizeof(Packet)<4096,"One bounded actual-device T3 value operation");
__global__ void Startup(Packet* p) { p->status=port::InitializeReference(p->input,p->reference); }
__global__ void Rates(Packet* p) { p->status=port::EvaluatePrescribed(p->reference,p->interval,p->result); }
class T3PortCuda : public ::testing::Test {
 protected:
  Packet* device=nullptr;
  void SetUp() override {
    int count=0; ASSERT_EQ(cudaGetDeviceCount(&count),cudaSuccess); ASSERT_GT(count,0);
    ASSERT_EQ(cudaGetLastError(),cudaSuccess); ASSERT_EQ(cudaMalloc(&device,sizeof(Packet)),cudaSuccess);
  }
  void TearDown() override { if(device) EXPECT_EQ(cudaFree(device),cudaSuccess); }
  void Run(Packet& p,bool startup) {
    ASSERT_EQ(cudaMemcpy(device,&p,sizeof(p),cudaMemcpyHostToDevice),cudaSuccess);
    if(startup) Startup<<<1,1>>>(device); else Rates<<<1,1>>>(device);
    ASSERT_EQ(cudaGetLastError(),cudaSuccess); ASSERT_EQ(cudaDeviceSynchronize(),cudaSuccess);
    ASSERT_EQ(cudaMemcpy(&p,device,sizeof(p),cudaMemcpyDeviceToHost),cudaSuccess);
  }
};
}
TEST_F(T3PortCuda, ActualStartupMatchesNativeShapesScalesRotatedAndEdgeOnFrames) {
  for(double scale:{1.,.02,1e-5}) for(unsigned shape=0;shape<3;++shape) for(unsigned orientation=0;orientation<3;++orientation) {
    Packet p; p.input=Triangle(scale,shape); p.input.node_ids[2]=(1ULL<<48)+37;
    for(auto& x:p.input.position) {
      if(orientation==1)x=kt::Rotate(kt::Rotation(),x);
      if(orientation==2)x={x.y,x.z,x.x};
      if(scale==1e-5) { x.x+=5; x.y-=3; x.z+=.125; }
    }
    native::Reference truth; ASSERT_EQ(native::Initialize(Native(p.input),truth),native::Status::kSuccess);
    const auto in=Bytes(p.input); ASSERT_NO_FATAL_FAILURE(Run(p,true));
    ASSERT_EQ(p.status,port::Status::kSuccess); EXPECT_EQ(Bytes(p.input),in);
    ASSERT_NO_FATAL_FAILURE(StartupAgreement(p.reference,truth));
  }
  RecordProperty("owned_device_bytes",static_cast<int>(sizeof(Packet)));
  RecordProperty("device_allocations",1); RecordProperty("threads_per_block",1);
}
TEST_F(T3PortCuda, ActualRatesMatchAllNativeColumnsAndChangedCurrentGeometry) {
  for(unsigned variant=0;variant<3;++variant) {
    Packet p; p.input=Triangle(.02); ASSERT_NO_FATAL_FAILURE(Run(p,true)); ASSERT_EQ(p.status,port::Status::kSuccess);
    native::Reference truth; ASSERT_EQ(native::Initialize(Native(p.input),truth),native::Status::kSuccess);
    for(unsigned column=0;column<18;++column) {
      p.interval=Interval(p.input,.0025); p.interval.base_time=.125; p.interval.sample_index=(1ULL<<53)+17;
      Set(column%6<3?p.interval.velocity[column/6]:p.interval.angular_velocity[column/6],column%3,.125);
      for(unsigned n=0;n<3;++n) {
        auto transform=[&](port::Vec3 x) { return variant==1?port::Vec3{x.y,x.z,x.x}:kt::Rotate(kt::Rotation(),x); };
        if(variant) { p.interval.position[n]=transform(p.interval.position[n]); p.interval.velocity[n]=transform(p.interval.velocity[n]);
          p.interval.angular_velocity[n]=transform(p.interval.angular_velocity[n]); }
      }
      if(variant==2) p.interval.position[2].z+=.003;
      native::Kinematics expected; ASSERT_EQ(native::EvaluatePrescribed(truth,Native(p.interval),expected),native::Status::kSuccess);
      const auto reference_bytes=Bytes(p.reference); ASSERT_NO_FATAL_FAILURE(Run(p,false));
      ASSERT_EQ(p.status,port::Status::kSuccess); EXPECT_EQ(Bytes(p.reference),reference_bytes);
      ASSERT_NO_FATAL_FAILURE(RatesAgreement(p.result,expected,p.interval));
    }
  }
}
TEST_F(T3PortCuda, DeviceCutoffLateArithmeticAndMalformedFailuresPreserveBytesAndRetry) {
  Packet p; p.input=Triangle(.02); ASSERT_NO_FATAL_FAILURE(Run(p,true)); ASSERT_EQ(p.status,port::Status::kSuccess);
  const auto good=p.input; const auto reference_bytes=Bytes(p.reference);
  for(unsigned fault=0;fault<4;++fault) {
    p.input=good;
    if(fault==0)p.input=Triangle(port::detail::MinimumEdge,1);
    if(fault==1)p.input.position[1]=p.input.position[0];
    if(fault==2)p.input.thickness=std::numeric_limits<double>::denorm_min();
    if(fault==3) { p.input.density=std::numeric_limits<double>::max(); p.input.thickness=100; }
    ASSERT_NO_FATAL_FAILURE(Run(p,true)); EXPECT_NE(p.status,port::Status::kSuccess); EXPECT_EQ(Bytes(p.reference),reference_bytes);
  }
  p.input=good; ASSERT_NO_FATAL_FAILURE(Run(p,true)); ASSERT_EQ(p.status,port::Status::kSuccess);
  const auto rate_reference_bytes=Bytes(p.reference);
  p.interval=Interval(good); p.interval.velocity[2].z=.125; p.interval.angular_velocity[1].x=.0625;
  const auto good_interval=p.interval; ASSERT_NO_FATAL_FAILURE(Run(p,false)); ASSERT_EQ(p.status,port::Status::kSuccess);
  const auto result_bytes=Bytes(p.result); const auto clean=p.result;
  for(unsigned fault=0;fault<5;++fault) {
    p.interval=good_interval;
    if(fault==0)p.interval.dt=0;
    if(fault==1)p.interval.velocity[2].z=std::numeric_limits<double>::max();
    if(fault==2)p.interval.angular_velocity[2].x=std::numeric_limits<double>::quiet_NaN();
    if(fault==3)p.interval.position[2]=p.interval.position[1];
    if(fault==4) { p.interval.position[0]={0,0,0}; p.interval.position[1]={4e-9,0,0};
      p.interval.position[2]={2e-9,32*port::detail::Em15+.25*port::detail::GuardBand*4e-9,0}; }
    ASSERT_NO_FATAL_FAILURE(Run(p,false)); EXPECT_NE(p.status,port::Status::kSuccess);
    EXPECT_EQ(Bytes(p.result),result_bytes); EXPECT_EQ(Bytes(p.reference),rate_reference_bytes);
  }
  p.interval=good_interval; ASSERT_NO_FATAL_FAILURE(Run(p,false)); ASSERT_EQ(p.status,port::Status::kSuccess);
  ExactRates(p.result,clean);
}
} // namespace t3_port_test
