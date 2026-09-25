// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Packet.h"
#include "lib_src/collision/RadiossType25AssemblyDevice.h"
#include "../radioss_type25_friction/CudaFixture.h"
#include <memory>
namespace type25_assembly_test {
constexpr unsigned MaxRows = 4097, CorpusRows = 257, CorpusNodes = 93;
struct IncidenceCase {
  ass::Connectivity rows[MaxRows];
  std::uint32_t ends[MaxRows];
  ass::NativeEndpoints packets[CorpusRows];
  ass::NativeNodalValue incoming[CorpusNodes], staged[CorpusNodes];
  ass::Status status[CorpusNodes];
};
__global__ void GatherWithBuiltIncidence(IncidenceCase* c, ass::Incidence incidence) {
  const auto node = blockIdx.x*blockDim.x+threadIdx.x;
  if (node < CorpusNodes) c->status[node] = ass::GatherNode(node,c->rows,c->packets,
      {c->ends,5,CorpusRows},incidence,c->incoming[node],&c->staged[node]);
}
__global__ void EmptyKernelForLaunchFailure() {}
using Type25DeviceIncidence = type25_friction_test::FrictionCuda;
static ass::IncidenceLimits Limits() { return {MaxRows,251,MaxRows,16u<<20}; }
static ass::DeviceConnectivity Current(IncidenceCase* d, std::size_t rows=CorpusRows,
    std::size_t nodes=CorpusNodes, std::size_t cohorts=5) {
  return {rows?d->rows:nullptr,{cohorts?d->ends:nullptr,cohorts,rows},nodes,{1,2,3,4,5}};
}
static void UploadCase(const Case& c, IncidenceCase& data) {
  std::copy(c.rows.begin(),c.rows.end(),data.rows);
  std::copy(c.cohorts.begin(),c.cohorts.end(),data.ends);
  std::copy(c.incoming.begin(),c.incoming.end(),data.incoming);
  for (unsigned row=0;row<CorpusRows;++row)
    ASSERT_EQ(ass::PrepareNativeEndpoints(Controls(),c.responses[row],&data.packets[row]),ass::Status::Ok);
}
static void SameCsr(ass::Incidence actual, const IncidenceCase& host,
    std::size_t rows, std::size_t nodes, std::size_t cohorts) {
  std::vector<std::uint32_t> offsets(nodes+1),ranks(5*rows),gpu_offsets(nodes+1),gpu_ranks(5*rows);
  ASSERT_TRUE(ass::BuildIncidence(rows?host.rows:nullptr,{cohorts?host.ends:nullptr,cohorts,rows},
      nodes,offsets.data(),offsets.size(),ranks.empty()?nullptr:ranks.data(),ranks.size()));
  ASSERT_EQ(actual.node_count,nodes); ASSERT_EQ(actual.occurrence_count,ranks.size());
  ASSERT_EQ(cudaMemcpy(gpu_offsets.data(),actual.offsets,gpu_offsets.size()*sizeof(std::uint32_t),
      cudaMemcpyDeviceToHost),cudaSuccess);
  if (!ranks.empty()) ASSERT_EQ(cudaMemcpy(gpu_ranks.data(),actual.occurrences,
      gpu_ranks.size()*sizeof(std::uint32_t),cudaMemcpyDeviceToHost),cudaSuccess);
  EXPECT_EQ(gpu_offsets,offsets); EXPECT_EQ(gpu_ranks,ranks);
}
TEST_F(Type25DeviceIncidence, DeviceCsrAndCompleteNativeGatherKeepEveryOrderedOccurrence) {
  static_assert(sizeof(IncidenceCase)<Capacity*RowBytes);
  const auto c=Corpus(); auto data=std::make_unique<IncidenceCase>();
  type25_friction_test::Drain drain{stream}; UploadCase(c,*data);
  auto* device=static_cast<IncidenceCase*>(input);
  ASSERT_EQ(cudaMemcpyAsync(device,data.get(),sizeof(*data),cudaMemcpyHostToDevice,stream),cudaSuccess);
  ass::DeviceIncidenceBuilder builder;
  ASSERT_EQ(builder.Initialize(Limits(),stream),ass::IncidenceStatus::Ok);
  ASSERT_EQ(builder.Stage(Current(device)),ass::IncidenceStatus::Ok);
  const auto view=builder.view(); ASSERT_TRUE(builder.IsCurrent(view));
  SameCsr(view.incidence(),*data,CorpusRows,CorpusNodes,5);
  GatherWithBuiltIncidence<<<2,64,0,stream>>>(device,view.incidence());
  ASSERT_EQ(cudaGetLastError(),cudaSuccess);
  ASSERT_EQ(cudaMemcpyAsync(data.get(),device,sizeof(*data),cudaMemcpyDeviceToHost,stream),cudaSuccess);
  ASSERT_EQ(cudaStreamSynchronize(stream),cudaSuccess);
  const auto native=NativeAssemble(c.rows,c.responses,c.cohorts,c.incoming);
  for(unsigned n=0;n<CorpusNodes;++n) {
    ASSERT_EQ(data->status[n],ass::Status::Ok); Same(data->staged[n],native[n]);
  }
  EXPECT_EQ(builder.last_report().sort_calls,1u); EXPECT_EQ(builder.last_report().host_fences,1u);
}
TEST_F(Type25DeviceIncidence, ChangedAssociationsCohortsAndDenseRepeatedNodesRemainComplete) {
  auto data=std::make_unique<IncidenceCase>(); type25_friction_test::Drain drain{stream};
  auto* device=static_cast<IncidenceCase*>(input); ass::DeviceIncidenceBuilder builder;
  ASSERT_EQ(builder.Initialize(Limits(),stream),ass::IncidenceStatus::Ok);
  for(unsigned rows:{1u,31u,256u,257u,4097u})for(bool dense:{false,true}) {
    for(unsigned row=0;row<rows;++row) {
      data->rows[row]={{dense?0:row%251,(row*17)%251,(row*19)%251,(row*19)%251},row%251};
      data->ends[row]=row+1;
    }
    const auto previous=builder.view();
    ASSERT_EQ(cudaMemcpyAsync(device,data.get(),sizeof(*data),cudaMemcpyHostToDevice,stream),cudaSuccess);
    auto current=Current(device,rows,251,rows); current.stamp.associations=rows+1;
    ASSERT_EQ(builder.Stage(current),ass::IncidenceStatus::Ok);
    EXPECT_FALSE(builder.IsCurrent(previous)); SameCsr(builder.view().incidence(),*data,rows,251,rows);
  }
}
TEST_F(Type25DeviceIncidence, BadCohortAndNodeRejectAllStagingWithDeterministicFailureAndRetry) {
  const auto c=Corpus(); auto data=std::make_unique<IncidenceCase>();
  type25_friction_test::Drain drain{stream}; UploadCase(c,*data);
  auto* device=static_cast<IncidenceCase*>(input); ass::DeviceIncidenceBuilder builder;
  ASSERT_EQ(builder.Initialize(Limits(),stream),ass::IncidenceStatus::Ok);
  for(unsigned repeat=0;repeat<8;++repeat) {
    data->ends[1]=0; data->rows[3].main[0]=UINT32_MAX;
    ASSERT_EQ(cudaMemcpyAsync(device,data.get(),sizeof(*data),cudaMemcpyHostToDevice,stream),cudaSuccess);
    EXPECT_EQ(builder.Stage(Current(device)),ass::IncidenceStatus::InvalidInput);
    EXPECT_EQ(builder.last_report().bad_cohort,1u); EXPECT_FALSE(builder.IsCurrent(builder.view()));
    data->ends[1]=c.cohorts[1];
    ASSERT_EQ(cudaMemcpyAsync(device,data.get(),sizeof(*data),cudaMemcpyHostToDevice,stream),cudaSuccess);
    EXPECT_EQ(builder.Stage(Current(device)),ass::IncidenceStatus::InvalidInput);
    ass::Occurrence at; const auto bad=builder.last_report().bad_occurrence;
    ASSERT_LT(bad,5*CorpusRows);
    ASSERT_TRUE(ass::DecodeOccurrence({data->ends,5,CorpusRows},bad,&at));
    EXPECT_EQ(at.row,3u); EXPECT_EQ(at.slot,0u);
    data->rows[3]=c.rows[3];
    ASSERT_EQ(cudaMemcpyAsync(device,data.get(),sizeof(*data),cudaMemcpyHostToDevice,stream),cudaSuccess);
    ASSERT_EQ(builder.Stage(Current(device)),ass::IncidenceStatus::Ok);
    SameCsr(builder.view().incidence(),*data,CorpusRows,CorpusNodes,5);
  }
}
TEST_F(Type25DeviceIncidence, HostAdmissionAliasAndCapsExpireViewsWithoutReallocation) {
  const auto c=Corpus(); auto data=std::make_unique<IncidenceCase>();
  type25_friction_test::Drain drain{stream}; UploadCase(c,*data);
  auto* device=static_cast<IncidenceCase*>(input); ass::DeviceIncidenceBuilder builder;
  EXPECT_EQ(builder.Stage(Current(device)),ass::IncidenceStatus::NotInitialized);
  ASSERT_EQ(builder.Initialize(Limits(),stream),ass::IncidenceStatus::Ok);
  const auto bytes=builder.allocations().device_bytes;
  EXPECT_EQ(builder.Initialize(Limits(),stream),ass::IncidenceStatus::AlreadyInitialized);
  ASSERT_EQ(cudaMemcpyAsync(device,data.get(),sizeof(*data),cudaMemcpyHostToDevice,stream),cudaSuccess);
  ASSERT_EQ(builder.Stage(Current(device)),ass::IncidenceStatus::Ok);
  const auto good=builder.view();
  auto bad=Current(device); bad.schedule.cohort_ends=good.incidence().offsets;
  EXPECT_EQ(builder.Stage(bad),ass::IncidenceStatus::InvalidInput); EXPECT_FALSE(builder.IsCurrent(good));
  bad=Current(device); bad.schedule.row_count=MaxRows+1;
  EXPECT_EQ(builder.Stage(bad),ass::IncidenceStatus::ResourceLimit);
  bad=Current(device); bad.stamp.attempt=0;
  EXPECT_EQ(builder.Stage(bad),ass::IncidenceStatus::InvalidInput);
  ASSERT_EQ(builder.Stage(Current(device)),ass::IncidenceStatus::Ok);
  const auto retried=builder.view(); builder.Discard(); EXPECT_FALSE(builder.IsCurrent(retried));
  EXPECT_EQ(builder.allocations().device_bytes,bytes);
  ass::IncidenceForecast forecast{17,19,23}; auto limits=Limits(); limits.max_device_bytes=1;
  EXPECT_EQ(ass::DeviceIncidenceBuilder::Preflight(limits,forecast),ass::IncidenceStatus::ResourceLimit);
  EXPECT_EQ(forecast.device_bytes,17u); EXPECT_EQ(forecast.cub_bytes,19u); EXPECT_EQ(forecast.host_bytes,23u);
}
TEST_F(Type25DeviceIncidence, EmptyContactAndIndependentAcceptedTrialStorage) {
  const auto c=Corpus(); auto data=std::make_unique<IncidenceCase>();
  type25_friction_test::Drain drain{stream}; UploadCase(c,*data);
  auto* device=static_cast<IncidenceCase*>(input);
  ass::DeviceIncidenceBuilder accepted,trial,empty;
  ASSERT_EQ(accepted.Initialize(Limits(),stream),ass::IncidenceStatus::Ok);
  ASSERT_EQ(trial.Initialize(Limits(),stream),ass::IncidenceStatus::Ok);
  ASSERT_EQ(empty.Initialize({0,CorpusNodes,0,4096},stream),ass::IncidenceStatus::Ok);
  ASSERT_EQ(cudaMemcpyAsync(device,data.get(),sizeof(*data),cudaMemcpyHostToDevice,stream),cudaSuccess);
  ASSERT_EQ(accepted.Stage(Current(device)),ass::IncidenceStatus::Ok);
  const auto accepted_view=accepted.view();
  auto bad=Current(device); bad.stamp.topology=0;
  EXPECT_EQ(trial.Stage(bad),ass::IncidenceStatus::InvalidInput);
  EXPECT_TRUE(accepted.IsCurrent(accepted_view)); EXPECT_FALSE(trial.IsCurrent(accepted_view));
  SameCsr(accepted_view.incidence(),*data,CorpusRows,CorpusNodes,5);
  ASSERT_EQ(empty.Stage(Current(device,0,CorpusNodes,0)),ass::IncidenceStatus::Ok);
  SameCsr(empty.view().incidence(),*data,0,CorpusNodes,0);
  EXPECT_EQ(empty.last_report().sort_calls,0u);
}
TEST_F(Type25DeviceIncidence, PriorCudaLaunchErrorPoisonsWorkspaceWithoutClaimingUnlaunchedWork) {
  const auto c=Corpus(); auto data=std::make_unique<IncidenceCase>();
  type25_friction_test::Drain drain{stream}; UploadCase(c,*data);
  auto* device=static_cast<IncidenceCase*>(input); ass::DeviceIncidenceBuilder builder;
  ASSERT_EQ(builder.Initialize(Limits(),stream),ass::IncidenceStatus::Ok);
  ASSERT_EQ(cudaMemcpyAsync(device,data.get(),sizeof(*data),cudaMemcpyHostToDevice,stream),cudaSuccess);
  ASSERT_EQ(builder.Stage(Current(device)),ass::IncidenceStatus::Ok);
  const auto previous=builder.view();
  // A zero-grid launch fails admission without executing a device instruction
  // or poisoning the CUDA context. Stage must consume, report and retain failure.
  EmptyKernelForLaunchFailure<<<0,1,0,stream>>>();
  ASSERT_EQ(cudaPeekAtLastError(),cudaErrorInvalidConfiguration);
  EXPECT_EQ(builder.Stage(Current(device)),ass::IncidenceStatus::DeviceFailure);
  EXPECT_FALSE(builder.IsCurrent(previous));
  EXPECT_EQ(builder.last_report().own_kernel_launches,0u);
  EXPECT_EQ(builder.last_report().sort_calls,0u); EXPECT_EQ(builder.last_report().host_fences,1u);
  EXPECT_EQ(builder.Stage(Current(device)),ass::IncidenceStatus::Unusable);
  EXPECT_EQ(cudaGetLastError(),cudaSuccess);
}
} // namespace type25_assembly_test
