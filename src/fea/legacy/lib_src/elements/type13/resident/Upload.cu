// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Storage.h"
namespace tl::fea::type13 {
BatchReport Batch::Impl::Upload(util::HostArena& arena,
    const batch_detail::Storage& header) {
  auto report = PendingError();
  if (!report) {
    return report;
  }
  report = Runtime(cudaMalloc(reinterpret_cast<void**>(&device), layout.bytes),
                         "TYPE13 device arena allocation failed");
  if (!report) {
    return report;
  }
  auto uploaded_header = batch_detail::RebasedHeader(device, layout);
  uploaded_header.model.config = header.model.config;
  uploaded_header.model.units = header.model.units;
  uploaded_header.model.source_instance_id = header.model.source_instance_id;
  uploaded_header.model.element_count = header.model.element_count;
  uploaded_header.control = header.control;
  uploaded_header.assembly.touched_count = header.assembly.touched_count;
  device_header = uploaded_header;
  *util::ArenaPointer<batch_detail::Storage>(arena.data(), layout.header) = uploaded_header;
  report = Runtime(cudaMemcpy(device, arena.data(), layout.bytes,
                                    cudaMemcpyHostToDevice), "TYPE13 startup upload failed");
  if (!report) {
    return report;
  }
  return {};
}
} // namespace tl::fea::type13
