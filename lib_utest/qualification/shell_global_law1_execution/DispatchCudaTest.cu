#include "../shell_global_law1/Fixture.h"
#include "lib_src/elements/qeph/QephBatchLayeredSection.h"
#include "lib_src/elements/t3/T3BatchLayeredSection.h"
#include "lib_src/elements/qeph/mapped/Stiffness.h"
#include "lib_src/elements/t3/mapped/Stiffness.h"
#include <cuda_runtime.h>
namespace global_law1_execution_dispatch_test {
using namespace global_law1_test;
namespace storage=tl::fea::shell_batch_plasticity_detail;
struct Packet {
  q::ReferenceData qr;t::ReferenceData tr;q::History qh;t::History th;
  q::PrescribedInterval qi;t::PrescribedInterval ti;
  q::ForceTrial qo;t::ForceTrial to;q::Status qs;t::Status ts;
  q::mapped::NodalStiffness qinitial;t::mapped::NodalStiffness tinitial;bool qinitial_ok=false,tinitial_ok=false;
  tl::fea::ShellSectionLaw law[1]{tl::fea::ShellSectionLaw::GlobalLaw1Npt0};Profile profile[1];bool present=true;
  tl::material::ShellElasticLaw1PointParameters parameters[1];
  tl::fea::sections::ShellLayeredLaw1History elastic[2][1];tl::fea::ShellBatchSectionState plastic[2][1];
};
__global__ void Dispatch(Packet* p) {
  storage::MixedDeviceStorage view;view.law=p->law;view.global_law1=p->present?p->profile:nullptr;
  view.elastic_parameters=p->parameters;
  for(unsigned slab=0;slab<2;++slab){view.elastic_section[slab]=p->elastic[slab];view.plastic.section[slab]=p->plastic[slab];}
  p->qs=q::batch_detail::EvaluateMixedSection(p->qr,p->qh,p->qi,view,0,0,p->qo);
  p->ts=t::batch_detail::EvaluateMixedSection(p->tr,p->th,p->ti,view,0,0,p->to);
  p->qinitial_ok=q::mapped::InitialStiffness(p->qr,p->law[0],p->qinitial,view.global_law1);
  p->tinitial_ok=t::mapped::InitialStiffness(p->tr,p->law[0],p->tinitial,view.global_law1);
}
struct Device {
  Packet* data=nullptr;cudaStream_t stream=nullptr;
  ~Device(){if(stream)cudaStreamSynchronize(stream);if(data)cudaFree(data);if(stream)cudaStreamDestroy(stream);}
};
TEST(GlobalLaw1ExecutionCuda,GlobalDispatchAndInitialPacketsConsumeProfileWithoutPointHistories) {
  for(double length:{1.,.001}) {
    Packet value{};Device device;
    ASSERT_EQ(cudaStreamCreateWithFlags(&device.stream,cudaStreamNonBlocking),cudaSuccess);
    ASSERT_EQ(cudaMalloc(&device.data,sizeof(Packet)),cudaSuccess);
    auto qr=Quad();auto tr=Triangle();qr.thickness=tr.thickness=.5*global::NativeEm20()*length;
    ASSERT_EQ(q::InitializeReference(qr,value.qr),q::Status::kSuccess);
    ASSERT_EQ(t::InitializeReference(tr,value.tr),t::Status::kSuccess);
    value.qh=QHistory(value.qr);value.th=THistory(value.tr);value.profile[0]=Accepted(length);
    value.qi=qeph_force_port_test::Next(qr,value.qh);value.ti=t3_port_test::Interval(tr,1e-6);value.ti.sample_index=1;
    const auto poison=std::numeric_limits<double>::quiet_NaN();value.parameters[0].young_pa=poison;
    for(auto& slab:value.elastic)for(auto& point:slab[0].point)for(auto& stress:point.stress)stress=poison;
    for(auto& slab:value.plastic)for(auto& point:slab[0].history.point)for(auto& stress:point.stress)stress=poison;
    q::ForceTrial qexpected;t::ForceTrial texpected;
    ASSERT_EQ(q::EvaluateGlobalLaw1Force(value.profile[0],value.qr,value.qh,value.qi,qexpected),q::Status::kSuccess);
    ASSERT_EQ(t::EvaluateGlobalLaw1Force(value.profile[0],value.tr,value.th,value.ti,texpected),t::Status::kSuccess);
    ASSERT_EQ(cudaMemcpyAsync(device.data,&value,sizeof value,cudaMemcpyHostToDevice,device.stream),cudaSuccess);
    Dispatch<<<1,1,0,device.stream>>>(device.data);ASSERT_EQ(cudaGetLastError(),cudaSuccess);
    ASSERT_EQ(cudaMemcpyAsync(&value,device.data,sizeof value,cudaMemcpyDeviceToHost,device.stream),cudaSuccess);
    ASSERT_EQ(cudaStreamSynchronize(device.stream),cudaSuccess);
    ASSERT_EQ(value.qs,q::Status::kSuccess);ASSERT_EQ(value.ts,t::Status::kSuccess);
    ForceBits(value.qo,qexpected);ForceBits(value.to,texpected);
    EXPECT_TRUE(value.qinitial_ok);EXPECT_TRUE(value.tinitial_ok);
    q::mapped::NodalStiffness qinitial;t::mapped::NodalStiffness tinitial;
    ASSERT_TRUE(q::mapped::InitialStiffness(value.qr,value.law[0],qinitial,value.profile));
    ASSERT_TRUE(t::mapped::InitialStiffness(value.tr,value.law[0],tinitial,value.profile));
    for(unsigned i=0;i<4;++i){DoubleBits(value.qinitial.translation[i],qinitial.translation[i]);DoubleBits(value.qinitial.rotation[i],qinitial.rotation[i]);}
    for(unsigned i=0;i<3;++i){DoubleBits(value.tinitial.translation[i],tinitial.translation[i]);DoubleBits(value.tinitial.rotation[i],tinitial.rotation[i]);}
    EXPECT_TRUE(std::isnan(value.elastic[0][0].point[0].stress[0]));EXPECT_TRUE(std::isnan(value.plastic[1][0].history.point[0].stress[0]));
    const auto qbefore=Bytes(value.qo);const auto tbefore=Bytes(value.to);value.present=false;
    ASSERT_EQ(cudaMemcpyAsync(device.data,&value,sizeof value,cudaMemcpyHostToDevice,device.stream),cudaSuccess);
    Dispatch<<<1,1,0,device.stream>>>(device.data);ASSERT_EQ(cudaGetLastError(),cudaSuccess);
    ASSERT_EQ(cudaMemcpyAsync(&value,device.data,sizeof value,cudaMemcpyDeviceToHost,device.stream),cudaSuccess);
    ASSERT_EQ(cudaStreamSynchronize(device.stream),cudaSuccess);
    EXPECT_EQ(value.qs,q::Status::kInvalidInput);EXPECT_EQ(value.ts,t::Status::kInvalidInput);
    EXPECT_FALSE(value.qinitial_ok);EXPECT_FALSE(value.tinitial_ok);EXPECT_EQ(Bytes(value.qo),qbefore);EXPECT_EQ(Bytes(value.to),tbefore);
  }
}
} // namespace global_law1_execution_dispatch_test
