// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Packet.h"
#include "../radioss_type25_friction/CudaFixture.h"
#include <memory>
namespace type25_assembly_test {
constexpr unsigned Rows = 257, Nodes = 93;
struct DeviceCase {
  ass::Connectivity rows[Rows];
  rd::NativeFrictionResult responses[Rows];
  ass::NativeEndpoints packets[Rows];
  ass::SiEndpoints si_packets[Rows];
  std::uint32_t cohorts[5], offsets[Nodes+1], ranks[5*Rows];
  ass::NativeNodalValue incoming[Nodes], staged[Nodes];
  ass::SiNodalValue si_incoming[Nodes], si_staged[Nodes];
  ass::Status preparation[Rows], gather[Nodes];
};
__global__ void PrepareRows(DeviceCase* c) {
  const auto row = blockIdx.x * blockDim.x + threadIdx.x;
  if (row < Rows) c->preparation[row] = ass::PrepareNativeEndpoints({0,0,0,0,0,{0,0,0}},
      c->responses[row],&c->packets[row]);
}
__global__ void GatherNodes(DeviceCase* c) {
  const auto node = blockIdx.x * blockDim.x + threadIdx.x;
  if (node < Nodes) c->gather[node] = ass::GatherNode(node,c->rows,c->packets,
      {c->cohorts,5,Rows},{c->offsets,c->ranks,Nodes,5*Rows},c->incoming[node],&c->staged[node]);
}
__global__ void ConvertAndGatherSi(DeviceCase* c, bool convert) {
  const auto index = blockIdx.x * blockDim.x + threadIdx.x;
  if (convert) {
    if (index < Rows) c->preparation[index] = ass::EndpointsToSi({.125,16.,.5},
        c->packets[index],&c->si_packets[index]);
  } else if (index < Nodes) c->gather[index] = ass::GatherNode(index,c->rows,c->si_packets,
      {c->cohorts,5,Rows},{c->offsets,c->ranks,Nodes,5*Rows},c->si_incoming[index],&c->si_staged[index]);
}
using Type25AssemblyCuda = type25_friction_test::FrictionCuda;
static void Initialize(const Case& c, DeviceCase& data) {
  std::copy(c.rows.begin(),c.rows.end(),data.rows);
  std::copy(c.responses.begin(),c.responses.end(),data.responses);
  std::copy(c.cohorts.begin(),c.cohorts.end(),data.cohorts);
  std::copy(c.incoming.begin(),c.incoming.end(),data.incoming);
  ASSERT_TRUE(ass::BuildIncidence(data.rows,{data.cohorts,5,Rows},Nodes,
      data.offsets,Nodes+1,data.ranks,5*Rows));
}
TEST_F(Type25AssemblyCuda, ParallelNativeProductsAndGatherMatchCompleteFortranOutputs) {
  static_assert(sizeof(DeviceCase) < Capacity*RowBytes);
  const auto c = Corpus(); const auto native = NativeAssemble(c.rows,c.responses,c.cohorts,c.incoming);
  auto data = std::make_unique<DeviceCase>();
  type25_friction_test::Drain drain{stream};
  Initialize(c,*data);
  auto* device = static_cast<DeviceCase*>(input);
  for (unsigned repetition = 0; repetition < 16; ++repetition) {
    ASSERT_EQ(cudaMemcpyAsync(device,data.get(),sizeof(DeviceCase),cudaMemcpyHostToDevice,stream),cudaSuccess);
    const auto threads = repetition % 2 ? 64u : 128u;
    PrepareRows<<<(Rows+threads-1)/threads,threads,0,stream>>>(device);
    ASSERT_EQ(cudaGetLastError(),cudaSuccess);
    GatherNodes<<<(Nodes+threads-1)/threads,threads,0,stream>>>(device);
    ASSERT_EQ(cudaGetLastError(),cudaSuccess);
    ASSERT_EQ(cudaMemcpyAsync(data.get(),device,sizeof(DeviceCase),cudaMemcpyDeviceToHost,stream),cudaSuccess);
    ASSERT_EQ(cudaStreamSynchronize(stream),cudaSuccess);
    for (unsigned row = 0; row < Rows; ++row) ASSERT_EQ(data->preparation[row],ass::Status::Ok);
    for (unsigned node = 0; node < Nodes; ++node) {
      ASSERT_EQ(data->gather[node],ass::Status::Ok); Same(data->staged[node],native[node]);
    }
  }
}
TEST_F(Type25AssemblyCuda, LateInvalidPacketDoesNotPublishNodeAndRetryIsClean) {
  const auto c = Corpus(); auto data = std::make_unique<DeviceCase>();
  type25_friction_test::Drain drain{stream};
  Initialize(c,*data);
  for (unsigned row = 0; row < Rows; ++row)
    ASSERT_EQ(ass::PrepareNativeEndpoints(Controls(),c.responses[row],&data->packets[row]),ass::Status::Ok);
  constexpr auto failed_row = Rows-2;
  ASSERT_TRUE(data->packets[failed_row].active);
  const auto saved_packet = data->packets[failed_row];
  const auto node = data->rows[failed_row].secondary;
  const ass::NativeNodalValue sentinel{{999.,-0.,777.},666.};
  for (auto& value : data->staged) value = sentinel;
  data->packets[failed_row].secondary_resultant.x = std::numeric_limits<double>::infinity();
  auto* device = static_cast<DeviceCase*>(input);
  ASSERT_EQ(cudaMemcpyAsync(device,data.get(),sizeof(DeviceCase),cudaMemcpyHostToDevice,stream),cudaSuccess);
  GatherNodes<<<2,64,0,stream>>>(device);
  ASSERT_EQ(cudaGetLastError(),cudaSuccess);
  ASSERT_EQ(cudaMemcpyAsync(data.get(),device,sizeof(DeviceCase),cudaMemcpyDeviceToHost,stream),cudaSuccess);
  ASSERT_EQ(cudaStreamSynchronize(stream),cudaSuccess);
  ASSERT_EQ(data->gather[node],ass::Status::InvalidInput); Same(data->staged[node],sentinel);
  // Other successful nodes are still private staging, not physical publication.
  for (unsigned n = 0; n < Nodes; ++n) Same(data->incoming[n],c.incoming[n]);
  data->packets[failed_row] = saved_packet;
  ASSERT_EQ(cudaMemcpyAsync(device,data.get(),sizeof(DeviceCase),cudaMemcpyHostToDevice,stream),cudaSuccess);
  GatherNodes<<<1,128,0,stream>>>(device);
  ASSERT_EQ(cudaGetLastError(),cudaSuccess);
  ASSERT_EQ(cudaMemcpyAsync(data.get(),device,sizeof(DeviceCase),cudaMemcpyDeviceToHost,stream),cudaSuccess);
  ASSERT_EQ(cudaStreamSynchronize(stream),cudaSuccess);
  const auto native = NativeAssemble(c.rows,c.responses,c.cohorts,c.incoming);
  for (unsigned n = 0; n < Nodes; ++n) {
    ASSERT_EQ(data->gather[n],ass::Status::Ok); Same(data->staged[n],native[n]);
  }
}
TEST_F(Type25AssemblyCuda, DeviceSiBoundaryPreservesAllComponentsAndDyadicNativeScaling) {
  static_assert(sizeof(DeviceCase) < Capacity*RowBytes);
  const auto c = Corpus(); auto data = std::make_unique<DeviceCase>();
  type25_friction_test::Drain drain{stream};
  Initialize(c,*data);
  for (unsigned node = 0; node < Nodes; ++node) {
    const auto& in = c.incoming[node];
    data->si_incoming[node] = {{in.force.x*8.,in.force.y*8.,in.force.z*8.},in.stiffness*64.};
  }
  auto* device = static_cast<DeviceCase*>(input);
  ASSERT_EQ(cudaMemcpyAsync(device,data.get(),sizeof(DeviceCase),cudaMemcpyHostToDevice,stream),cudaSuccess);
  PrepareRows<<<3,128,0,stream>>>(device);
  ASSERT_EQ(cudaGetLastError(),cudaSuccess);
  ConvertAndGatherSi<<<3,128,0,stream>>>(device,true);
  ASSERT_EQ(cudaGetLastError(),cudaSuccess);
  ConvertAndGatherSi<<<1,128,0,stream>>>(device,false);
  ASSERT_EQ(cudaGetLastError(),cudaSuccess);
  ASSERT_EQ(cudaMemcpyAsync(data.get(),device,sizeof(DeviceCase),cudaMemcpyDeviceToHost,stream),cudaSuccess);
  ASSERT_EQ(cudaStreamSynchronize(stream),cudaSuccess);
  const auto native = NativeAssemble(c.rows,c.responses,c.cohorts,c.incoming);
  for (unsigned row = 0; row < Rows; ++row) {
    ASSERT_EQ(data->preparation[row],ass::Status::Ok);
    ass::SiEndpoints expected;
    ASSERT_EQ(ass::EndpointsToSi({.125,16.,.5},data->packets[row],&expected),ass::Status::Ok);
    const auto& actual = data->si_packets[row];
    EXPECT_EQ(actual.active,expected.active);
    Same({actual.secondary_resultant,actual.secondary_stiffness},
        {expected.secondary_resultant,expected.secondary_stiffness});
    for (unsigned slot = 0; slot < 4; ++slot)
      Same({actual.main_force[slot],actual.main_stiffness[slot]},
          {expected.main_force[slot],expected.main_stiffness[slot]});
  }
  for (unsigned node = 0; node < Nodes; ++node) {
    ASSERT_EQ(data->gather[node],ass::Status::Ok);
    Same({data->si_staged[node].force,data->si_staged[node].stiffness},
        {{native[node].force.x*8.,native[node].force.y*8.,native[node].force.z*8.},native[node].stiffness*64.});
  }
}
} // namespace type25_assembly_test
