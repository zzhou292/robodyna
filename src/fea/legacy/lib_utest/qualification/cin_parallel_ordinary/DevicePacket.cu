// SPDX-License-Identifier: AGPL-3.0-or-later
#include "DevicePacket.h"
#include <stdexcept>
#include <type_traits>
#include <utility>

namespace tl::fea::cin_parallel_test {
namespace {
void Check(cudaError_t error) {
  if (error != cudaSuccess) throw std::runtime_error(cudaGetErrorString(error));
}
template<class T> void Read(std::vector<T>& target, const T* source) {
  if (!target.empty()) Check(cudaMemcpy(target.data(), source, target.size()*sizeof(T), cudaMemcpyDeviceToHost));
}
}
void DeviceDelete::operator()(void* pointer) const noexcept {
  cudaFree(pointer);
}
void* DevicePacket::Upload(const void* source, std::size_t bytes) {
  if (!bytes) return nullptr;
  void* next = nullptr;
  Check(cudaMalloc(&next, bytes));
  std::unique_ptr<void, DeviceDelete> allocation(next);
  allocations_.push_back(std::move(allocation));
  Check(cudaMemcpy(next, source, bytes, cudaMemcpyHostToDevice));
  return next;
}
DevicePacket::DevicePacket(Packet& p, bool parallel_inputs, bool parallel_screen) {
  const auto upload = [this](const auto& values) {
    using T = typename std::decay_t<decltype(values)>::value_type;
    return static_cast<T*>(Upload(values.data(), values.size()*sizeof(T)));
  };
  input_ = p.Input();
  input_.control = static_cast<nodal_detail::Control*>(Upload(&p.control, sizeof(p.control)));
  input_.failure = static_cast<cin_advance::FailureKey*>(Upload(&p.failure, sizeof(p.failure)));
  if (parallel_inputs) {
    input_.input_failure = static_cast<cin_advance::FailureKey*>(Upload(&p.input_failure, sizeof(p.input_failure)));
  }
  if (parallel_screen) {
    const std::vector<cin_advance::screen::Summary> summaries(
        cin_advance::screen::Blocks(Nodes), {-7, 123, 456});
    input_.screen = upload(summaries);
  }
  input_.accepted = upload(p.accepted);
  input_.trial = upload(p.trial);
  input_.tail = input_.trial+p.TailOffset();
  input_.loads = upload(p.loads);
  input_.work = upload(p.work);
  input_.fixed = upload(p.fixed);
  input_.rotation_present = upload(p.present);
  input_.model.rows = upload(p.rows);
  input_.model.dependent_nodes = upload(p.dependent);
  input_.model.first_witness = upload(p.first_witness);
  input_.activity = upload(p.activity);
  input_.patches = upload(p.patches);
  input_.groups.groups = upload(p.groups);
  input_.groups.members = upload(p.members);
  input_.groups.member_nodes = p.groups.empty()?nullptr:upload(p.member_nodes);
  if (p.capture_enabled) {
    auto* capture = upload(p.capture);
    input_.capture = {capture, capture+3*Nodes, capture+6*Nodes,
      capture+6*Nodes+3*p.groups.size()};
  }
}
void DevicePacket::Run(bool parallel) {
  Check(parallel ? cin_advance::Launch(input_, nullptr) : LaunchSerial(input_, nullptr));
  Check(cudaStreamSynchronize(nullptr));
}
void DevicePacket::RunWith(cudaError_t (*launch)(const cin_advance::Input&, cudaStream_t)) {
  Check(launch(input_, nullptr));
  Check(cudaStreamSynchronize(nullptr));
}
cin_advance::screen::Summary DevicePacket::ScreenSummary() const {
  cin_advance::screen::Summary result{};
  Check(cudaMemcpy(&result, input_.screen, sizeof(result), cudaMemcpyDeviceToHost));
  return result;
}
void DevicePacket::Download(Packet& p) const {
  if (input_.input_failure) {
    Check(cudaMemcpy(&p.input_failure, input_.input_failure, sizeof(p.input_failure), cudaMemcpyDeviceToHost));
  }
  Read(p.accepted, input_.accepted);
  Read(p.trial, input_.trial);
  Read(p.loads, input_.loads);
  Read(p.work, input_.work);
  Read(p.patches, input_.patches);
  if (p.capture_enabled) Read(p.capture, input_.capture.node);
  Check(cudaMemcpy(&p.control, input_.control, sizeof(p.control), cudaMemcpyDeviceToHost));
  Check(cudaMemcpy(&p.failure, input_.failure, sizeof(p.failure), cudaMemcpyDeviceToHost));
}
} // namespace tl::fea::cin_parallel_test
