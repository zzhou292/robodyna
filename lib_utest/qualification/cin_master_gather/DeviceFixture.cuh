// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Fixture.h"
#include "Frozen.h"
#include "../cin_parallel_ordinary/DevicePacket.h"
#include <cuda_runtime.h>

namespace tl::fea::cin_gather_test {
// Test-only upload of the same bounded topology layout as actual owner startup.
// Original fields belong to DevicePacket; no accepted destination is replaced.
inline cudaError_t WithGather(const cin_advance::Input& original,cudaStream_t stream) {
  const auto n=original.model.node_count,r=original.model.row_count;
  if (!n || !r) return cin_advance::Launch(original,stream);
  std::vector<cin::StageRow> rows(r);
  std::vector<std::uint8_t> dependent(n);
  auto error=cudaMemcpy(rows.data(),original.model.rows,r*sizeof(cin::StageRow),cudaMemcpyDeviceToHost);
  if (error!=cudaSuccess) return error;
  error=cudaMemcpy(dependent.data(),original.model.dependent_nodes,n,cudaMemcpyDeviceToHost);
  if (error!=cudaSuccess) return error;
  auto host_input=original;
  host_input.model.rows=rows.data();host_input.model.dependent_nodes=dependent.data();
  Workspace host;
  if (!host.Initialize(host_input)) return cin_advance::Launch(original,stream);
  util::BoundedArenaLayout prefix(128u<<20);
  util::ArenaRegion guard,prepared;
  if (!prefix.Append<std::byte>(64,guard) ||
      !prefix.Append<cin::detail::PreparedForceRow>(r,prepared)) return cudaErrorInvalidValue;
  gather::Layout layout;
  if (!layout.Initialize(prefix.bytes(),n,r,128u<<20,128u<<20)) return cudaErrorInvalidValue;
  void* arena=nullptr;
  error=cudaMalloc(&arena,layout.device_bytes+64);
  if (error!=cudaSuccess) return error;
  const auto finish=[&](cudaError_t status) {
    const auto released=cudaFree(arena);
    return status==cudaSuccess?released:status;
  };
  error=cudaMemsetAsync(arena,0xa5,layout.device_bytes+64,stream);
  if (error!=cudaSuccess) return finish(error);
  auto input=original;
  input.prepared_transfers=util::ArenaPointer<cin::detail::PreparedForceRow>(arena,prepared);
  input.force_gather={original.model.rows,util::ArenaPointer<std::uint32_t>(arena,layout.nodes),
      util::ArenaPointer<std::uint32_t>(arena,layout.offsets),
      util::ArenaPointer<std::uint32_t>(arena,layout.incidence),
      util::ArenaPointer<gather::Master>(arena,layout.values),
      util::ArenaPointer<gather::Summary>(arena,layout.summary),
      n,r,host.view.master_count,std::uint32_t(layout.capacity)};
  for (const auto pair:{std::pair{layout.nodes,&host.nodes},std::pair{layout.offsets,&host.offsets},
                       std::pair{layout.incidence,&host.incidence}}) {
    error=cudaMemcpyAsync(util::ArenaPointer<std::uint32_t>(arena,pair.first),
        pair.second->data(),pair.first.bytes,cudaMemcpyHostToDevice,stream);
    if (error!=cudaSuccess) return finish(error);
  }
  error=cin_advance::Launch(input,stream);
  if (error!=cudaSuccess) return finish(error);
  error=cudaStreamSynchronize(stream);
  if (error!=cudaSuccess) return finish(error);
  std::array<unsigned char,64> before{},after{};
  error=cudaMemcpy(before.data(),arena,64,cudaMemcpyDeviceToHost);
  if (error!=cudaSuccess) return finish(error);
  error=cudaMemcpy(after.data(),static_cast<unsigned char*>(arena)+layout.device_bytes,64,cudaMemcpyDeviceToHost);
  if (error!=cudaSuccess) return finish(error);
  for (unsigned index=0;index<64;++index) {EXPECT_EQ(before[index],0xa5);EXPECT_EQ(after[index],0xa5);}
  return finish(cudaSuccess);
}
inline void RunPair(packet::Packet& serial,packet::Packet& actual,bool screen) {
  packet::DevicePacket before(serial,true,screen),after(actual,true,screen);
  before.RunWith(LaunchFrozen);after.RunWith(WithGather);
  before.Download(serial);after.Download(actual);
}
} // namespace tl::fea::cin_gather_test
