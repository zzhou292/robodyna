#include "ShellOnePointStorage.h"
#include "ShellOnePointValues.h"

namespace tl::fea::shell_batch_plasticity_detail {
OnePointHostStorage::~OnePointHostStorage() {
  if (device_) cudaFree(device_);
}
bool OnePointHostStorage::Forecast(std::size_t count, std::size_t device_cap,
    std::size_t host_cap, OnePointLayout& output, std::size_t& host_bytes) noexcept {
  OnePointLayout layout;
  util::BoundedArenaLayout host(host_cap);
  util::ArenaRegion ignored;
  if (!host_cap || host_cap > MaxVehicleShellResidentHostBytes ||
      !layout.Initialize(count, device_cap) ||
      !host.Append<unsigned char>(sizeof(OnePointHostStorage), ignored) ||
      !host.Append<unsigned char>(layout.bytes, ignored) ||
      !host.Append<ShellBatchOnePointSectionState>(count, ignored) ||
      !host.Append<unsigned char>(64, ignored)) return false;
  output = layout;
  host_bytes = host.bytes();
  return true;
}
SetupReport OnePointHostStorage::Initialize(const ShellBatchPlasticityBinding& catalog,
    const ShellBatchBinding& binding, std::size_t count, const OnePointLayout& layout) {
  if (device_ || !catalog.Matches(binding) || count != binding.t3_count() ||
      count != layout.section[0].count) {
    return {SetupStatus::InvalidInput, "One-point state requires complete T3 scope/layout"};
  }
  util::HostArena arena;
  if (!arena.Initialize(layout.bytes)) {
    return {SetupStatus::ResourceLimit, "One-point startup staging allocation failed"};
  }
  auto* initial = layout.Construct(arena);
  if (!initial) return {SetupStatus::ResourceLimit, "Invalid one-point startup layout"};
  staging_.Resize(count);
  for (std::size_t e = 0; e < count; ++e) {
    ShellSectionLaw law = ShellSectionLaw::Unspecified;
    if (!catalog.Law(ShellBindingFamily::T3, e, &law)) {
      return {SetupStatus::InvalidInput, "One-point startup source identity is incomplete"};
    }
    if (law != ShellSectionLaw::Law44Nip1) continue;
    for (auto* slab : initial->section) {
      slab[e].point.reported_thickness_m = binding.t3_reference(e).input.thickness;
    }
  }
  OnePointDeviceStorage* candidate = nullptr;
  auto error = cudaMalloc(reinterpret_cast<void**>(&candidate), layout.bytes);
  if (error != cudaSuccess) {
    return {SetupStatus::DeviceFailure, "One-point device allocation failed", error};
  }
  const auto header = layout.Rebase(candidate);
  *initial = header;
  error = cudaMemcpy(candidate, arena.data(), layout.bytes, cudaMemcpyHostToDevice);
  if (error != cudaSuccess) {
    cudaFree(candidate);
    return {SetupStatus::DeviceFailure, "One-point startup copy failed", error};
  }
  device_ = candidate;
  header_ = header;
  layout_ = layout;
  count_ = count;
  return {SetupStatus::Success, "OK"};
}
SetupReport OnePointHostStorage::Read(unsigned slab, std::size_t count, cudaStream_t stream,
    const ShellBatchPlasticityBinding& catalog, double time) noexcept {
  if (!device_ || slab > 1 || count != count_) {
    return {SetupStatus::InvalidInput, "One-point readback shape is invalid"};
  }
  auto error = cudaMemcpyAsync(staging_.data(), header_.section[slab],
      count * sizeof(ShellBatchOnePointSectionState), cudaMemcpyDeviceToHost, stream);
  if (error == cudaSuccess) error = cudaStreamSynchronize(stream);
  if (error != cudaSuccess) {
    return {SetupStatus::DeviceFailure, "One-point readback failed", error};
  }
  for (std::size_t e = 0; e < count; ++e) {
    ShellSectionLaw law = ShellSectionLaw::Unspecified;
    if (!catalog.Law(ShellBindingFamily::T3, e, &law)) {
      return {SetupStatus::InvalidInput, "One-point readback source identity is incomplete"};
    }
    if (law != ShellSectionLaw::Law44Nip1) continue;
    sections::PointParameters parameters;
    if (!catalog.Parameters(ShellBindingFamily::T3, e, &parameters) ||
        !ValidOnePointState(staging_[e], parameters, time)) {
      return {SetupStatus::NonfiniteResult, "One-point saved/current/failure state is invalid"};
    }
  }
  return {SetupStatus::Success, "OK"};
}
} // namespace tl::fea::shell_batch_plasticity_detail
