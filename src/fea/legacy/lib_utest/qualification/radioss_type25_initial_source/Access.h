// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "lib_src/collision/radioss_type25/initial_source/Internal.h"
#include <vector>
#include <stdexcept>
namespace tlfea::contact::radioss_type25::qualification {
// Owning expected-data readback only; no mutator or public production escape.
struct InitialSourceAccess {
  struct Result {
    std::vector<NativeGeometryHistory> history;
    std::vector<int> flags;
    std::vector<double> gaps;
  };
  static const initial_source::detail::Prepared& Host(const initial_source::PreparedSource& source) {
    if(!source.impl_)throw std::logic_error("Missing prepared source");
    return source.impl_->source;
  }
  static const initial_source::detail::Layout& StoredLayout(const initial_source::PreparedSource& source) {
    if(!source.impl_)throw std::logic_error("Missing prepared source");
    return source.impl_->layout;
  }
  static initial_source::Limits StoredLimits(const initial_source::PreparedSource& source) {
    if(!source.impl_)throw std::logic_error("Missing prepared source");
    return source.impl_->limits;
  }
  static Result Read(const initial_source::DeviceSeed& source,cudaStream_t stream) {
    if(!source.impl_)throw std::logic_error("Missing private initial seed");
    const auto& p=*source.impl_;Result out;
    out.history.resize(p.identity.secondaries);out.flags.resize(p.identity.secondaries);out.gaps.resize(4*p.identity.mains);
    cudaError_t e=cudaSuccess;
    const auto copy=[&](void* to,const tl::util::ArenaRegion& region) {
      if(e==cudaSuccess&&region.bytes)e=cudaMemcpyAsync(to,tl::util::ArenaPointer<std::byte>(p.data,region),region.bytes,cudaMemcpyDeviceToHost,stream);
    };
    copy(out.history.data(),p.layout.history);copy(out.flags.data(),p.layout.flags);copy(out.gaps.data(),p.layout.corner_gaps);
    const auto drained=cudaStreamSynchronize(stream);
    if(e!=cudaSuccess||drained!=cudaSuccess)throw std::runtime_error("Initial seed readback failed");
    return out;
  }
};
}
