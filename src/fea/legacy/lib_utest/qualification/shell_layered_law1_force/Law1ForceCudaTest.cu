#include "Law1ForceTestSupport.h"
#include <cuda_runtime_api.h>
namespace layered_law1_force_test {
struct QPacket {q::ReferenceData reference;Parameters material;q::LayeredLaw1History history;q::PrescribedInterval input;q::LayeredLaw1ForceTrial trial;q::Status status;};
struct TPacket {t::ReferenceData reference;Parameters material;t::LayeredLaw1History history;t::PrescribedInterval input;t::LayeredLaw1ForceTrial trial;t::Status status;};
__global__ void EvaluateQ(QPacket* p) {p->status=q::EvaluateLayeredLaw1Force(p->reference,p->material,p->history,p->input,p->trial);}
__global__ void EvaluateT(TPacket* p) {p->status=t::EvaluateLayeredLaw1Force(p->reference,p->material,p->history,p->input,p->trial);}
template<class P> struct Device {P* p=nullptr;~Device(){if(p)cudaFree(p);}};
TEST(LayeredLaw1ForceCuda,QephNativeLoadedRotationUnloadingForcesWorkAndRollback) {
  auto input=qeph_startup_test::Case(5);QPacket packet{};packet.material=Material(input);
  nq::Reference reference;native::QephHistory history;
  ASSERT_EQ(q::InitializeReference(input,packet.reference),q::Status::kSuccess);
  ASSERT_EQ(q::InitializeLayeredLaw1History(packet.reference,packet.material,{},packet.history),q::Status::kSuccess);
  ASSERT_EQ(nq::Initialize(qeph_startup_test::NativeInput(input),reference),nq::Status::kSuccess);
  ASSERT_EQ(nq::InitializeHistory(reference,{},history.shell),nq::Status::kSuccess);
  Device<QPacket> device;ASSERT_EQ(cudaMalloc(reinterpret_cast<void**>(&device.p),sizeof packet),cudaSuccess);
  path::Path motion{true,1};motion.angular_speed=18000;
  for(;motion.step<160;++motion.step) {
    SCOPED_TRACE(motion.step);packet.input=motion.Interval(input);native::QephTrial expected;
    ASSERT_EQ(native::Evaluate(reference,history,qeph_kinematics_test::NativeInterval(packet.input),expected),nq::Status::kSuccess);
    ASSERT_EQ(cudaMemcpy(device.p,&packet,sizeof packet,cudaMemcpyHostToDevice),cudaSuccess);
    EvaluateQ<<<1,1>>>(device.p);ASSERT_EQ(cudaGetLastError(),cudaSuccess);
    ASSERT_EQ(cudaMemcpy(&packet,device.p,sizeof packet,cudaMemcpyDeviceToHost),cudaSuccess);
    ASSERT_EQ(packet.status,q::Status::kSuccess);
    qeph_force_port_test::ForceAgreement(packet.trial.force,expected.shell,input,packet.input,path::cv::Tolerance);
    Points(packet.trial.proposed_section,expected.points);ASSERT_FALSE(::testing::Test::HasFailure());
    packet.history={packet.trial.force.proposed_history,packet.trial.proposed_section};
    history={expected.shell.proposed_history,expected.points};
  }
  const auto before=Bytes(packet.trial);const auto accepted=Bytes(packet.history);
  packet.input=motion.Interval(input);++packet.input.sample_index;
  ASSERT_EQ(cudaMemcpy(device.p,&packet,sizeof packet,cudaMemcpyHostToDevice),cudaSuccess);
  EvaluateQ<<<1,1>>>(device.p);ASSERT_EQ(cudaGetLastError(),cudaSuccess);
  ASSERT_EQ(cudaMemcpy(&packet,device.p,sizeof packet,cudaMemcpyDeviceToHost),cudaSuccess);
  EXPECT_EQ(packet.status,q::Status::kInvalidInput);EXPECT_EQ(Bytes(packet.trial),before);EXPECT_EQ(Bytes(packet.history),accepted);
  packet.input=motion.Interval(input);native::QephTrial retry;
  ASSERT_EQ(native::Evaluate(reference,history,qeph_kinematics_test::NativeInterval(packet.input),retry),nq::Status::kSuccess);
  ASSERT_EQ(cudaMemcpy(device.p,&packet,sizeof packet,cudaMemcpyHostToDevice),cudaSuccess);
  EvaluateQ<<<1,1>>>(device.p);ASSERT_EQ(cudaGetLastError(),cudaSuccess);
  ASSERT_EQ(cudaMemcpy(&packet,device.p,sizeof packet,cudaMemcpyDeviceToHost),cudaSuccess);
  ASSERT_EQ(packet.status,q::Status::kSuccess);
  qeph_force_port_test::ForceAgreement(packet.trial.force,retry.shell,input,packet.input,path::cv::Tolerance);
  Points(packet.trial.proposed_section,retry.points);
}
TEST(LayeredLaw1ForceCuda,T3NativeLoadedRotationUnloadingForcesWorkAndRollback) {
  auto input=t3_port_test::Triangle(.02,1);TPacket packet{};packet.material=Material(input,10e9);
  nt::Reference reference;native::T3History history;
  ASSERT_EQ(t::InitializeReference(input,packet.reference),t::Status::kSuccess);
  ASSERT_EQ(t::InitializeLayeredLaw1History(packet.reference,packet.material,{},packet.history),t::Status::kSuccess);
  ASSERT_EQ(nt::Initialize(t3_port_test::Native(input),reference),nt::Status::kSuccess);
  ASSERT_EQ(nt::InitializeHistory(reference,{},history.shell),nt::Status::kSuccess);
  Device<TPacket> device;ASSERT_EQ(cudaMalloc(reinterpret_cast<void**>(&device.p),sizeof packet),cudaSuccess);
  path::Path motion{true,1};motion.angular_speed=18000;
  for(;motion.step<160;++motion.step) {
    SCOPED_TRACE(motion.step);packet.input=motion.Interval(input);native::T3Trial expected;
    ASSERT_EQ(native::Evaluate(reference,history,t3_port_test::Native(packet.input),expected),nt::Status::kSuccess);
    ASSERT_EQ(cudaMemcpy(device.p,&packet,sizeof packet,cudaMemcpyHostToDevice),cudaSuccess);
    EvaluateT<<<1,1>>>(device.p);ASSERT_EQ(cudaGetLastError(),cudaSuccess);
    ASSERT_EQ(cudaMemcpy(&packet,device.p,sizeof packet,cudaMemcpyDeviceToHost),cudaSuccess);
    ASSERT_EQ(packet.status,t::Status::kSuccess);
    t3_force_port_test::Agreement(packet.reference,packet.input,packet.trial.force,expected.shell,path::cv::Tolerance);
    Points(packet.trial.proposed_section,expected.points);ASSERT_FALSE(::testing::Test::HasFailure());
    packet.history={packet.trial.force.proposed_history,packet.trial.proposed_section};
    history={expected.shell.proposed_history,expected.points};
  }
  const auto before=Bytes(packet.trial);const auto accepted=Bytes(packet.history);
  packet.input=motion.Interval(input);++packet.input.sample_index;
  ASSERT_EQ(cudaMemcpy(device.p,&packet,sizeof packet,cudaMemcpyHostToDevice),cudaSuccess);
  EvaluateT<<<1,1>>>(device.p);ASSERT_EQ(cudaGetLastError(),cudaSuccess);
  ASSERT_EQ(cudaMemcpy(&packet,device.p,sizeof packet,cudaMemcpyDeviceToHost),cudaSuccess);
  EXPECT_EQ(packet.status,t::Status::kInvalidInput);EXPECT_EQ(Bytes(packet.trial),before);EXPECT_EQ(Bytes(packet.history),accepted);
  packet.input=motion.Interval(input);native::T3Trial retry;
  ASSERT_EQ(native::Evaluate(reference,history,t3_port_test::Native(packet.input),retry),nt::Status::kSuccess);
  ASSERT_EQ(cudaMemcpy(device.p,&packet,sizeof packet,cudaMemcpyHostToDevice),cudaSuccess);
  EvaluateT<<<1,1>>>(device.p);ASSERT_EQ(cudaGetLastError(),cudaSuccess);
  ASSERT_EQ(cudaMemcpy(&packet,device.p,sizeof packet,cudaMemcpyDeviceToHost),cudaSuccess);
  ASSERT_EQ(packet.status,t::Status::kSuccess);
  t3_force_port_test::Agreement(packet.reference,packet.input,packet.trial.force,retry.shell,path::cv::Tolerance);
  Points(packet.trial.proposed_section,retry.points);
}
} // namespace layered_law1_force_test
