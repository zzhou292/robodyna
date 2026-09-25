#include "CudaFixture.h"
#include "lib_src/elements/solids/resident/AssemblySerial.cuh"
#include <cstring>
namespace solid_parallel_test {
void Cuda(cudaError_t error) {
  if (error != cudaSuccess) throw std::runtime_error(cudaGetErrorString(error));
}
__global__ void Reference(d::Storage* state, fe::NodalAssemblyView view, fe::NodalCinAssemblyView cin) {
  d::assembly_serial::Assemble(state, 0, view, cin);
}
DevicePacket::DevicePacket(Packet& p):packet(p),seed(14*p.nodes) {
  try {
    Cuda(cudaStreamCreateWithFlags(&stream, cudaStreamNonBlocking));
    Cuda(cudaMalloc(reinterpret_cast<void**>(&storage), p.layout.bytes));
    Cuda(cudaMalloc(reinterpret_cast<void**>(&data), seed.size()*sizeof(double)));
    Cuda(cudaMalloc(reinterpret_cast<void**>(&records), sizeof(Records)));
    for (std::size_t i=6*p.nodes; i<seed.size(); ++i)
      seed[i]=i%3 ? -0.0 : .125;
    Upload(); Reset();
  } catch (...) { Release(); throw; }
}
void DevicePacket::Release() noexcept {
  if (stream) cudaStreamSynchronize(stream);
  if (records) cudaFree(records);
  if (data) cudaFree(data);
  if (storage) cudaFree(storage);
  if (stream) cudaStreamDestroy(stream);
}
DevicePacket::~DevicePacket() { Release(); }
void DevicePacket::Upload() {
  auto saved = packet.State();
  auto header = d::RebasedHeader(storage, packet.layout);
  header.config = saved.config;
  packet.State() = header;
  const auto result = cudaMemcpyAsync(storage, packet.arena.data(), packet.layout.bytes,
      cudaMemcpyHostToDevice, stream);
  // Restore only after the borrowed upload has drained, including on failure.
  const auto synced = cudaStreamSynchronize(stream);
  packet.State() = saved;
  Cuda(result); Cuda(synced);
}
void DevicePacket::Reset() {
  const auto n = packet.nodes;
  Records initial;
  const StreamDrain drain{stream};
  initial.result.attempt = 1;
  initial.bounds.node_count = initial.bounds.capacity = n;
  initial.bounds.attempt = 1;
  initial.bounds.initialized = initial.bounds.valid = true;
  Cuda(cudaMemcpyAsync(data, seed.data(), seed.size()*sizeof(double), cudaMemcpyHostToDevice, stream));
  Cuda(cudaMemcpyAsync(records, &initial, sizeof(initial), cudaMemcpyHostToDevice, stream));
  Cuda(cudaStreamSynchronize(stream));
  view = {};
  view.accepted.position_xyz = data;
  view.accepted.velocity_xyz = data+3*n;
  view.accepted.node_count = n;
  view.forces = {data+6*n,data+7*n,data+8*n,data+9*n,data+10*n,data+11*n,n,0};
  view.result = &records->result;
  view.bounds = &records->bounds;
  view.stream = stream;
  view.attempt = 1;
  cin = {};
  cin.translational_stiffness = data+12*n;
  cin.rotational_stiffness = data+13*n;
}
cudaError_t DevicePacket::Launch(bool serial) {
  if (serial) { Reference<<<1,1,0,stream>>>(storage,view,cin); return cudaPeekAtLastError(); }
  return d::LaunchAssembly(storage,0,view,cin);
}
Snapshot DevicePacket::Read() {
  Snapshot result;
  result.fields.resize(seed.size());
  const StreamDrain drain{stream};
  Cuda(cudaMemcpyAsync(result.fields.data(),data,seed.size()*sizeof(double),cudaMemcpyDeviceToHost,stream));
  Cuda(cudaMemcpyAsync(&result.records,records,sizeof(Records),cudaMemcpyDeviceToHost,stream));
  Cuda(cudaMemcpyAsync(&result.control,&storage->control,sizeof(d::Control),cudaMemcpyDeviceToHost,stream));
  Cuda(cudaMemcpyAsync(&result.fallback,&storage->assembly.fallback,sizeof(unsigned),cudaMemcpyDeviceToHost,stream));
  Cuda(cudaStreamSynchronize(stream));
  return result;
}
void Same(const Snapshot& a,const Snapshot& b) {
  ASSERT_EQ(a.fields.size(),b.fields.size());
  EXPECT_EQ(std::memcmp(a.fields.data(),b.fields.data(),a.fields.size()*sizeof(double)),0);
  EXPECT_EQ(a.control.status,b.control.status); EXPECT_EQ(a.control.family,b.control.family);
  EXPECT_EQ(a.control.parent,b.control.parent); EXPECT_EQ(a.control.node,b.control.node);
  EXPECT_EQ(a.control.element_status,b.control.element_status);
  EXPECT_TRUE(d::SameDiagnostics(a.control.diagnostics,b.control.diagnostics));
  const auto& x=a.records.result;const auto& y=b.records.result;
  EXPECT_EQ(x.status,y.status);EXPECT_EQ(x.node,y.node);EXPECT_EQ(x.base_epoch,y.base_epoch);EXPECT_EQ(x.attempt,y.attempt);
  const auto& u=a.records.bounds;const auto& v=b.records.bounds;
  EXPECT_EQ(u.stiffness,v.stiffness);EXPECT_EQ(u.damping,v.damping);
  EXPECT_EQ(u.node_count,v.node_count);EXPECT_EQ(u.capacity,v.capacity);
  EXPECT_EQ(u.base_epoch,v.base_epoch);EXPECT_EQ(u.attempt,v.attempt);
  EXPECT_EQ(u.initialized,v.initialized);EXPECT_EQ(u.valid,v.valid);EXPECT_EQ(u.sealed,v.sealed);
}
} // namespace solid_parallel_test
