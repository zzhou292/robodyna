// SPDX-License-Identifier: MIT
#include "Transfers.h"
#include "lib_src/elements/qeph/QephForceData.h"
#include "lib_src/elements/ShellBatchFailure.h"
#include "lib_src/elements/qeph/mapped/ActivityLayout.h"

namespace qeph_activity_test { Transfers transfers; }
extern "C" cudaError_t __real_cudaMemcpyAsync(void*,const void*,std::size_t,cudaMemcpyKind,cudaStream_t);
extern "C" cudaError_t __wrap_cudaMemcpyAsync(void* destination,const void* source,
    std::size_t bytes,cudaMemcpyKind kind,cudaStream_t stream) {
  auto& t = qeph_activity_test::transfers;
  if (t.enabled && kind == cudaMemcpyDeviceToHost) {
    ++t.calls;
    t.bytes += bytes;
    t.stream = stream;
    if (bytes == t.parents*sizeof(tl::fea::qeph::ForceTrial)) {
      t.force_source = source;
      ++t.force_calls;
    }
    if (bytes == t.parents*sizeof(tl::fea::ShellBatchFailureState)) t.failure_source = source;
    if (bytes == tl::fea::qeph::mapped::ActivityBytes(t.parents)) {
      t.compact_destination = destination;
      ++t.compact_calls;
    }
  }
  return __real_cudaMemcpyAsync(destination,source,bytes,kind,stream);
}
