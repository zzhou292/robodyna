#include "NativeOracle.h"
#include <cuda_runtime.h>
namespace law36_test {
namespace {
struct Packet {
  double x[8]{},y[8]{};
  law::Parameters parameters{};
  law::CallerHistory accepted{};
  law::Kinematics motion{};
  law::Measures measures{};
  law::CallerResult result{};
  law::Status status{};
};
struct Device {
  Packet* packet=nullptr;
  ~Device() { if (packet) cudaFree(packet); }
};
__global__ void Configure(Packet* p) {
  p->status=law::Prepare(E,Nu,Rho,{p->x,p->y,8},p->parameters);
}
__global__ void Advance(Packet* p) {
  p->status=law::UpdateCaller(p->parameters,p->accepted,p->motion,p->measures,p->result);
}
bool Send(Device& d,const Packet& p) {
  return cudaMemcpy(d.packet,&p,sizeof p,cudaMemcpyHostToDevice)==cudaSuccess;
}
bool Read(const Device& d,Packet& p) {
  return cudaMemcpy(&p,d.packet,sizeof p,cudaMemcpyDeviceToHost)==cudaSuccess;
}
bool Initialize(Device& d,Packet& p) {
  std::copy_n(X,8,p.x);
  std::copy_n(Y,8,p.y);
  if (cudaMalloc(reinterpret_cast<void**>(&d.packet),sizeof(Packet))!=cudaSuccess) return false;
  if (!Send(d,p)) return false;
  Configure<<<1,1>>>(d.packet);
  return cudaGetLastError()==cudaSuccess && Read(d,p) && p.status==law::Status::Ok;
}
}
TEST(SolidLaw36Cuda, DevicePreparedOriginalCallerWithIndependentNativeHistory) {
  int count=0;
  ASSERT_EQ(cudaGetDeviceCount(&count),cudaSuccess);
  ASSERT_GT(count,0);
  Device d;
  Packet p{};
  ASSERT_TRUE(Initialize(d,p));
  const auto parameters=Parameters();
  EXPECT_DOUBLE_EQ(p.parameters.bulk_pa,parameters.bulk_pa);
  EXPECT_DOUBLE_EQ(p.parameters.sound_speed_m_s,parameters.sound_speed_m_s);
  law::CallerHistory native{};
  unsigned plastic=0;
  for (unsigned step=0;step<280;++step) {
    SCOPED_TRACE(step);
    p.motion=Motion(step);
    p.measures=Measures(step);
    const auto expected=Native(parameters,native,p.motion,p.measures);
    ASSERT_TRUE(Send(d,p));
    Advance<<<1,1>>>(d.packet);
    ASSERT_EQ(cudaGetLastError(),cudaSuccess);
    ASSERT_TRUE(Read(d,p));
    ASSERT_EQ(p.status,law::Status::Ok);
    ASSERT_TRUE(Close(Values(p.result),expected));
    plastic+=p.result.point.plastic_increment>0;
    p.accepted=p.result.history;
    native=NativeHistory(expected);
  }
  EXPECT_GT(plastic,120u);
  EXPECT_GT(p.accepted.point.plastic_strain,.03);
}
TEST(SolidLaw36Cuda, StartupLateFailureAndRetryPreserveActualDevicePacket) {
  Device d;
  Packet p{};
  ASSERT_TRUE(Initialize(d,p));
  const auto parameters=Parameters();
  p.motion=Motion(0);
  p.motion.dt_s=0;
  p.measures={Rho,2e-8,2e-8,0,0,0};
  ASSERT_TRUE(Send(d,p));
  Advance<<<1,1>>>(d.packet);
  ASSERT_EQ(cudaGetLastError(),cudaSuccess);
  ASSERT_TRUE(Read(d,p));
  ASSERT_EQ(p.status,law::Status::Ok);
  ASSERT_TRUE(Close(Values(p.result),Native(parameters,{},p.motion,p.measures)));
  p.accepted=p.result.history;
  p.motion=Motion(0);
  p.measures=Measures(0);
  const auto expected=Native(parameters,p.accepted,p.motion,p.measures);
  ASSERT_TRUE(Send(d,p));
  Advance<<<1,1>>>(d.packet);
  ASSERT_EQ(cudaGetLastError(),cudaSuccess);
  ASSERT_TRUE(Read(d,p));
  ASSERT_EQ(p.status,law::Status::Ok);
  const auto saved=Bytes(p.result);
  const auto base=Bytes(p.accepted);
  p.motion.engineering_rate_per_s[5]=std::numeric_limits<double>::max();
  ASSERT_TRUE(Send(d,p));
  Advance<<<1,1>>>(d.packet);
  ASSERT_EQ(cudaGetLastError(),cudaSuccess);
  ASSERT_TRUE(Read(d,p));
  ASSERT_NE(p.status,law::Status::Ok);
  EXPECT_EQ(Bytes(p.result),saved);
  EXPECT_EQ(Bytes(p.accepted),base);
  p.motion=Motion(0);
  ASSERT_TRUE(Send(d,p));
  Advance<<<1,1>>>(d.packet);
  ASSERT_EQ(cudaGetLastError(),cudaSuccess);
  ASSERT_TRUE(Read(d,p));
  ASSERT_EQ(p.status,law::Status::Ok);
  EXPECT_EQ(Bytes(p.result),saved);
  EXPECT_EQ(Bytes(p.accepted),base);
  ASSERT_TRUE(Close(Values(p.result),expected));
}
}
