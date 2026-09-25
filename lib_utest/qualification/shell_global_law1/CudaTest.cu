#include "Fixture.h"
#include <cuda_runtime_api.h>
namespace global_law1_test {
struct QPacket {Profile profile;q::ReferenceData reference;q::History history;q::PrescribedInterval input;q::ForceTrial trial;q::Status status;};
struct TPacket {Profile profile;t::ReferenceData reference;t::History history;t::PrescribedInterval input;t::ForceTrial trial;t::Status status;};
__global__ void EvaluateQ(QPacket* p) {p->status=q::EvaluateGlobalLaw1Force(p->profile,p->reference,p->history,p->input,p->trial);}
__global__ void EvaluateT(TPacket* p) {p->status=t::EvaluateGlobalLaw1Force(p->profile,p->reference,p->history,p->input,p->trial);}
template<class Packet> struct Device {
  Packet* data=nullptr;cudaStream_t stream=nullptr;
  bool Initialize() {return cudaStreamCreateWithFlags(&stream,cudaStreamNonBlocking)==cudaSuccess&&cudaMalloc(reinterpret_cast<void**>(&data),sizeof(Packet))==cudaSuccess;}
  ~Device(){if(stream)cudaStreamSynchronize(stream);if(data)cudaFree(data);if(stream)cudaStreamDestroy(stream);}
};
#define CHECK_PACKET(KERNEL) do { \
  ASSERT_EQ(cudaMemcpyAsync(device.data,&packet,sizeof packet,cudaMemcpyHostToDevice,device.stream),cudaSuccess); \
  KERNEL<<<1,1,0,device.stream>>>(device.data);ASSERT_EQ(cudaGetLastError(),cudaSuccess); \
  ASSERT_EQ(cudaMemcpyAsync(&packet,device.data,sizeof packet,cudaMemcpyDeviceToHost,device.stream),cudaSuccess); \
  ASSERT_EQ(cudaStreamSynchronize(device.stream),cudaSuccess); \
} while(false)
TEST(GlobalLaw1Cuda,QephBothModesNativeRecurrenceExactHostParityAndRetry) {
  for(int ithk:{0,1}) {
    auto input=Quad();QPacket packet{};packet.profile={ithk?Thickness::Accepted:Thickness::Reference,1.};
    ASSERT_EQ(q::InitializeReference(input,packet.reference),q::Status::kSuccess);
    packet.history=QHistory(packet.reference);nq::Reference nr;
    ASSERT_EQ(nq::Initialize(qeph_startup_test::NativeInput(input),nr),nq::Status::kSuccess);
    auto history=QNativeHistory(nr,packet.history);Device<QPacket> device;ASSERT_TRUE(device.Initialize());
    path::Path motion{true,1};motion.angular_speed=18000;
    for(;motion.step<160;++motion.step) {
      SCOPED_TRACE(ithk);SCOPED_TRACE(motion.step);packet.input=motion.Interval(input);
      nq::ForceTrial expected;q::ForceTrial cpu;
      ASSERT_EQ(global::Evaluate(nr,history,qeph_kinematics_test::NativeInterval(packet.input),ithk,expected),nq::Status::kSuccess);
      ASSERT_EQ(q::EvaluateGlobalLaw1Force(packet.profile,packet.reference,packet.history,packet.input,cpu),q::Status::kSuccess);
      CHECK_PACKET(EvaluateQ);ASSERT_EQ(packet.status,q::Status::kSuccess);
      qeph_force_port_test::ForceAgreement(packet.trial,expected,input,packet.input,path::cv::Tolerance);
      qeph_force_port_test::ForceAgreement(packet.trial,cpu,input,packet.input,0.);ForceBits(packet.trial,cpu);
      ASSERT_FALSE(::testing::Test::HasFailure());
      packet.history=packet.trial.proposed_history;history=expected.proposed_history;
    }
    const auto before=Bytes(packet.trial);const auto old=Bytes(packet.history);
    packet.input=motion.Interval(input);packet.profile.thickness=static_cast<Thickness>(99);
    CHECK_PACKET(EvaluateQ);EXPECT_EQ(packet.status,q::Status::kInvalidInput);
    EXPECT_EQ(Bytes(packet.trial),before);EXPECT_EQ(Bytes(packet.history),old);
    packet.profile.thickness=ithk?Thickness::Accepted:Thickness::Reference;
    CHECK_PACKET(EvaluateQ);ASSERT_EQ(packet.status,q::Status::kSuccess);
    nq::ForceTrial retry;
    ASSERT_EQ(global::Evaluate(nr,history,qeph_kinematics_test::NativeInterval(packet.input),ithk,retry),nq::Status::kSuccess);
    qeph_force_port_test::ForceAgreement(packet.trial,retry,input,packet.input,path::cv::Tolerance);
  }
}
TEST(GlobalLaw1Cuda,QephNativeLengthFloorContexts) {
  auto input=Quad();QPacket packet{};
  ASSERT_EQ(q::InitializeReference(input,packet.reference),q::Status::kSuccess);
  Device<QPacket> device;ASSERT_TRUE(device.Initialize());
  for(double length:{1.,.001,.01}) {
    packet.profile=Accepted(length);const double h=.5*global::NativeEm20()*length;
    packet.history=QHistory(packet.reference,h);
    packet.input=qeph_force_port_test::Next(input,packet.history);packet.input.sample_index=1;
    q::ForceTrial cpu;
    ASSERT_EQ(q::EvaluateGlobalLaw1Force(packet.profile,packet.reference,packet.history,packet.input,cpu),q::Status::kSuccess);
    CHECK_PACKET(EvaluateQ);ASSERT_EQ(packet.status,q::Status::kSuccess);
    qeph_force_port_test::ForceAgreement(packet.trial,cpu,input,packet.input,0.);ForceBits(packet.trial,cpu);
    EXPECT_EQ(packet.trial.diagnostics.effective_thickness,global::NativeEm20()*length);
  }
}
TEST(GlobalLaw1Cuda,T3BothModesNativeRecurrenceExactHostParityAndRetry) {
  for(int ithk:{0,1}) {
    auto input=Triangle();TPacket packet{};packet.profile={ithk?Thickness::Accepted:Thickness::Reference,1.};
    ASSERT_EQ(t::InitializeReference(input,packet.reference),t::Status::kSuccess);
    packet.history=THistory(packet.reference);nt::Reference nr;
    ASSERT_EQ(nt::Initialize(t3_port_test::Native(input),nr),nt::Status::kSuccess);
    auto history=t3_force_port_test::Native(nr,packet.history);Device<TPacket> device;ASSERT_TRUE(device.Initialize());
    path::Path motion{true,1};motion.angular_speed=18000;
    for(;motion.step<160;++motion.step) {
      SCOPED_TRACE(ithk);SCOPED_TRACE(motion.step);packet.input=motion.Interval(input);
      nt::ForceTrial expected;t::ForceTrial cpu;
      ASSERT_EQ(global::Evaluate(nr,history,t3_port_test::Native(packet.input),ithk,expected),nt::Status::kSuccess);
      ASSERT_EQ(t::EvaluateGlobalLaw1Force(packet.profile,packet.reference,packet.history,packet.input,cpu),t::Status::kSuccess);
      CHECK_PACKET(EvaluateT);ASSERT_EQ(packet.status,t::Status::kSuccess);
      t3_force_port_test::Agreement(packet.reference,packet.input,packet.trial,expected,path::cv::Tolerance);
      t3_force_port_test::Exact(packet.trial,cpu);ForceBits(packet.trial,cpu);
      ASSERT_FALSE(::testing::Test::HasFailure());
      packet.history=packet.trial.proposed_history;history=expected.proposed_history;
    }
    const auto before=Bytes(packet.trial);const auto old=Bytes(packet.history);
    packet.input=motion.Interval(input);packet.profile.thickness=static_cast<Thickness>(99);
    CHECK_PACKET(EvaluateT);EXPECT_EQ(packet.status,t::Status::kInvalidInput);
    EXPECT_EQ(Bytes(packet.trial),before);EXPECT_EQ(Bytes(packet.history),old);
    packet.profile.thickness=ithk?Thickness::Accepted:Thickness::Reference;
    CHECK_PACKET(EvaluateT);ASSERT_EQ(packet.status,t::Status::kSuccess);
    nt::ForceTrial retry;
    ASSERT_EQ(global::Evaluate(nr,history,t3_port_test::Native(packet.input),ithk,retry),nt::Status::kSuccess);
    t3_force_port_test::Agreement(packet.reference,packet.input,packet.trial,retry,path::cv::Tolerance);
  }
}
TEST(GlobalLaw1Cuda,T3NativeLengthFloorContexts) {
  auto input=Triangle();TPacket packet{};
  ASSERT_EQ(t::InitializeReference(input,packet.reference),t::Status::kSuccess);
  Device<TPacket> device;ASSERT_TRUE(device.Initialize());
  for(double length:{1.,.001,.01}) {
    packet.profile=Accepted(length);const double h=.5*global::NativeEm20()*length;
    packet.history=THistory(packet.reference,h);
    packet.input=t3_port_test::Interval(input,1e-6);packet.input.sample_index=1;
    t::ForceTrial cpu;
    ASSERT_EQ(t::EvaluateGlobalLaw1Force(packet.profile,packet.reference,packet.history,packet.input,cpu),t::Status::kSuccess);
    CHECK_PACKET(EvaluateT);ASSERT_EQ(packet.status,t::Status::kSuccess);
    t3_force_port_test::Exact(packet.trial,cpu);ForceBits(packet.trial,cpu);
    EXPECT_EQ(packet.trial.diagnostics.effective_thickness,h);
  }
}
#undef CHECK_PACKET
} // namespace global_law1_test
