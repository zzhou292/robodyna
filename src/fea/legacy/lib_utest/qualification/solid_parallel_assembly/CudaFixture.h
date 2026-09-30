#pragma once
#include "Packet.h"
#include <gtest/gtest.h>
#include <cuda_runtime.h>
namespace solid_parallel_test {
struct StreamDrain {
  cudaStream_t stream;
  ~StreamDrain() { if (stream) cudaStreamSynchronize(stream); }
};
struct Records { fe::NodalAssemblyResult result; tl::fea::stability::RowBounds bounds; };
struct Snapshot {
  std::vector<double> fields;
  Records records;
  d::Control control;
  unsigned fallback = 0;
};
class DevicePacket {
 public:
  explicit DevicePacket(Packet& packet);
  ~DevicePacket();
  DevicePacket(const DevicePacket&) = delete;
  DevicePacket& operator=(const DevicePacket&) = delete;
  void Reset();
  cudaError_t Launch(bool serial);
  Snapshot Read();
  void Upload();
  Packet& packet;
  cudaStream_t stream = nullptr;
  d::Storage* storage = nullptr;
  double* data = nullptr;
  Records* records = nullptr;
  fe::NodalAssemblyView view;
  fe::NodalCinAssemblyView cin;
  std::vector<double> seed;
 private:
  void Release() noexcept;
};
void Same(const Snapshot& a, const Snapshot& b);
void Cuda(cudaError_t error);
} // namespace solid_parallel_test
