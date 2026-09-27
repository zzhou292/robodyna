// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Fixture.h"
#include "../cin_parallel_ordinary/DevicePacket.h"
#include <cstring>
#include <stdexcept>
namespace tl::fea::group_motion_test {
namespace {
unsigned capture_mode=0;
double source_units=.001;
double maximum_angle=.2;
std::vector<groups::Report> reports;
void Check(cudaError_t e) {if(e!=cudaSuccess)throw std::runtime_error(cudaGetErrorString(e));}
__device__ void Fail(cin_advance::Input in,NodalStatus status,unsigned node) {
  in.control->status=status;in.control->node=node;
  if(status==NodalStatus::StepTooLarge)in.control->limit.dt=0;
}
// The original, unchanged serial group helper is separately source-pinned.
// Full caller/owner tests below are compiled from their existing frozen package.
__global__ void SerialMotion(cin_advance::Input in) {
  if(in.control->status!=NodalStatus::Ok)return;
  const auto failure=*in.failure;
  if(failure!=cin_advance::NoFailure) {
    Fail(in,cin_advance::FailureStatus(failure),cin_advance::FailureNode(failure));return;
  }
  for(unsigned group=0;group<in.groups.group_count;++group) {
    const auto report=groups::AdvanceGroup(in,group);in.group_reports[group]=report;
    if(report.status!=NodalStatus::Ok) {Fail(in,report.status,report.last_node);return;}
  }
}
template<bool Frozen>
cudaError_t WithReports(const cin_advance::Input& original,cudaStream_t stream) {
  const auto count=original.groups.group_count;groups::Report* storage=nullptr;
  Check(cudaMalloc(reinterpret_cast<void**>(&storage),(count+2)*sizeof(*storage)));
  try {
    Check(cudaMemsetAsync(storage,0xa5,(count+2)*sizeof(*storage),stream));
    auto in=original;in.group_reports=storage+1;in.groups.source_length_to_m=source_units;in.maximum_angle=maximum_angle;
    if(capture_mode==1)in.capture.node=nullptr;
    if(capture_mode==2)in.capture.node_rotation=nullptr;
    if(capture_mode==3)in.capture.group=nullptr;
    if(capture_mode==4)in.capture.group_rotation=nullptr;
    nodal_detail::Control before;cin_advance::FailureKey prior;
    Check(cudaMemcpy(&before,in.control,sizeof(before),cudaMemcpyDeviceToHost));
    Check(cudaMemcpy(&prior,in.failure,sizeof(prior),cudaMemcpyDeviceToHost));
    if constexpr(Frozen) {SerialMotion<<<1,1,0,stream>>>(in);Check(cudaGetLastError());}
    else Check(groups::LaunchMotion(in,stream));
    Check(cudaStreamSynchronize(stream));
    std::vector<unsigned char> bytes((count+2)*sizeof(*storage));
    Check(cudaMemcpy(bytes.data(),storage,bytes.size(),cudaMemcpyDeviceToHost));
    for(unsigned a=0;a<sizeof(*storage);++a) {
      EXPECT_EQ(bytes[a],0xa5);EXPECT_EQ(bytes[(count+1)*sizeof(*storage)+a],0xa5);
    }
    reports.clear();
    if(before.status!=NodalStatus::Ok||prior!=cin_advance::NoFailure) {
      for(auto value:bytes)EXPECT_EQ(value,0xa5);
    } else {
      reports.resize(count);
      std::memcpy(reports.data(),bytes.data()+sizeof(*storage),count*sizeof(*storage));
    }
    Check(cudaFree(storage));
  } catch(...) {cudaFree(storage);throw;}
  return cudaSuccess;
}
void Compare(packet::Packet before,NodalStatus expected=NodalStatus::Ok) {
  auto old=before,now=before;
  packet::DevicePacket a(old),b(now);
  a.RunWith(WithReports<true>);const auto reference=reports;
  b.RunWith(WithReports<false>);
  ASSERT_EQ(reference.size(),reports.size());
  for(unsigned i=0;i<reports.size();++i)Same(reference[i],reports[i]);
  a.Download(old);b.Download(now);
  EXPECT_EQ(old.control.status,expected);packet::SameControl(old.control,now.control);
  EXPECT_EQ(old.failure,now.failure);SamePrivate(before,old,now);
}
}
TEST(CinCooperativeMotionCuda, ExactMotionAndCaptureAcrossAllMemberTileBoundaries) {
  capture_mode=0;
  for(bool capture:{false,true})for(unsigned count:{2,3,63,64,65,127,128,129,260}) {
    SCOPED_TRACE(capture);
    SCOPED_TRACE(count);
    auto p=Members({count});p.capture_enabled=capture;p.failure=cin_advance::NoFailure;
    for(double units:{.001,1.})for(double previous:{0.,packet::H}) {
      source_units=units;p.durations={previous,previous?packet::H:packet::H/2,packet::H};Compare(p);
    }
  }
  source_units=.001;auto p=Members({65,129,2});p.failure=cin_advance::NoFailure;Compare(p);
  p=Members({65});p.failure=cin_advance::NoFailure;
  std::reverse(p.members.begin(),p.members.end());Compare(p);
  std::fill(p.loads.begin(),p.loads.end(),-0.);Compare(p);
  p=Members({2});p.failure=cin_advance::NoFailure;
  p.members[1].mass=p.members[1].inertia=0;
  for(double units:{.001,1.,0.}) {
    source_units=units;Compare(p,units>0?NodalStatus::Ok:NodalStatus::InvalidOutput);
  }
  source_units=.001;
}
TEST(CinCooperativeMotionCuda, FailedTilesPreserveExactPrivatePrefixesAndCompetingErrorOrder) {
  capture_mode=0;source_units=.001;
  for(bool capture:{false,true})for(unsigned local:{0,63,64,65,128})for(unsigned fault=0;fault<8;++fault) {
    if(fault==2&&local<2)continue;
    SCOPED_TRACE(capture);
    SCOPED_TRACE(local);
    SCOPED_TRACE(fault);
    auto p=Members({129});p.capture_enabled=capture;Fault(p,fault,local);
    p.failure=cin_advance::NoFailure;Compare(p,NodalStatus::InvalidOutput);
  }
  auto p=Members({129});p.failure=cin_advance::NoFailure;
  maximum_angle=0;Compare(p,NodalStatus::StepTooLarge);maximum_angle=.2;
  // Clean fixture control; same-allocation repair/retry is covered by the reused owner test.
  Compare(p);
}
TEST(CinCooperativeMotionCuda, CaptureDispatchAndPriorFailuresLeaveReportsUntouched) {
  source_units=.001;
  for(capture_mode=0;capture_mode<5;++capture_mode) {
    auto p=Members({65});p.failure=cin_advance::NoFailure;
    Compare(p,capture_mode>1?NodalStatus::InvalidOutput:NodalStatus::Ok);
  }
  capture_mode=0;
  auto p=Members({65});p.failure=cin_advance::NoFailure;p.control.status=NodalStatus::InvalidOutput;
  Compare(p,NodalStatus::InvalidOutput);
  p.control.status=NodalStatus::Ok;p.failure=cin_advance::EncodeFailure(7,NodalStatus::StepTooLarge);
  Compare(p,NodalStatus::StepTooLarge);
}
} // namespace tl::fea::group_motion_test
