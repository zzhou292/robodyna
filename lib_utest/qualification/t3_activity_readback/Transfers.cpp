// SPDX-License-Identifier: MIT
#include "Transfers.h"
#include "lib_src/elements/t3/T3ForceData.h"
#include "lib_src/elements/ShellBatchOnePointSection.h"
#include <limits>

namespace t3_readback_test { Transfers transfers; }
extern "C" cudaError_t __real_cudaMemcpyAsync(void*, const void*, std::size_t, cudaMemcpyKind, cudaStream_t);
extern "C" cudaError_t __real_cudaStreamSynchronize(cudaStream_t);
extern "C" cudaError_t __wrap_cudaStreamSynchronize(cudaStream_t stream) {
  if (t3_readback_test::transfers.enabled) ++t3_readback_test::transfers.syncs;
  return __real_cudaStreamSynchronize(stream);
}
extern "C" cudaError_t __wrap_cudaMemcpyAsync(void* destination, const void* source,
    std::size_t bytes, cudaMemcpyKind kind, cudaStream_t stream) {
  using namespace t3_readback_test;
  auto& t = transfers;
  const bool watched = t.enabled && kind == cudaMemcpyDeviceToHost;
  if (watched) {
    ++t.copies;
    t.bytes += bytes;
    t.stream = stream;
    if (t.copies == t.fail_copy) return cudaErrorInvalidValue;
  }
  auto status = __real_cudaMemcpyAsync(destination, source, bytes, kind, stream);
  if (status != cudaSuccess || !watched) return status;
  if (bytes == t.parents * sizeof(tl::fea::t3::ForceTrial)) {
    ++t.force_copies;
    t.force_host = destination;
    t.force_device = source;
    if (t.fault == Fault::ForceNonfinite || t.fault == Fault::Both) {
      status = __real_cudaStreamSynchronize(stream);
      if (status == cudaSuccess) static_cast<tl::fea::t3::ForceTrial*>(destination)[t.parents - 1]
          .diagnostics.native_sound_speed = std::numeric_limits<double>::quiet_NaN();
    }
  } else if (t.copies == 1 && bytes == t.parents * sizeof(tl::fea::ShellBatchOnePointSectionState)) {
    if (t.fault == Fault::PointThickness || t.fault == Fault::Both || t.fault == Fault::RawPointFlag) {
      status = __real_cudaStreamSynchronize(stream);
      if (status == cudaSuccess) {
        auto& point = static_cast<tl::fea::ShellBatchOnePointSectionState*>(destination)[t.parents - 1];
        if (t.fault == Fault::RawPointFlag)
          *reinterpret_cast<unsigned char*>(&point.point.failure.history.point_active) = 255;
        else point.point.reported_thickness_m *= 2;
      }
    }
  }
  return status;
}
