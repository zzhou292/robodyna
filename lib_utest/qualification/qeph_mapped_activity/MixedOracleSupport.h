// SPDX-License-Identifier: MIT
#pragma once
#include "lib_src/elements/ShellBatchPlasticityStorage.h"
#include <cstring>
#include <vector>
namespace qeph_activity_test::frozen_mixed {
using namespace tl::fea;
using shell_batch_plasticity_detail::SetupReport;
using shell_batch_plasticity_detail::SetupStatus;
// Only transport/source-query seams are provided here. The complete frozen
// mixed reader and numerical predicates are compiled without body edits.
struct ShellBatchPlasticityBinding {
  std::vector<ShellSectionLaw> roles;
  bool execution = true;
  bool heterogeneous_sections() const { return true; }
  bool execution_sections() const { return execution; }
  bool Law(ShellBindingFamily,std::size_t parent,ShellSectionLaw* output) const {
    if (parent >= roles.size()) return false;
    *output = roles[parent];
    return true;
  }
};
inline constexpr auto cudaMemcpyAsync = [](void* output,const void* input,std::size_t bytes,
    cudaMemcpyKind,cudaStream_t) -> cudaError_t {
  std::memcpy(output,input,bytes);
  return cudaSuccess;
};
inline constexpr auto cudaStreamSynchronize = [](cudaStream_t) -> cudaError_t { return cudaSuccess; };
struct MixedHostStorage {
  bool device_ = true;
  std::size_t count_ = 0;
  ShellBindingFamily family_ = ShellBindingFamily::Qeph;
  std::vector<ShellBatchSectionState> plastic_;
  std::vector<sections::ShellLayeredLaw1History> elastic_;
  std::vector<ShellBatchLayeredSection> output_;
  struct Header {
    struct Plastic { const ShellBatchSectionState* section[2]{}; } plastic;
    const sections::ShellLayeredLaw1History* elastic_section[2]{};
  } header_;
  void Bind(const std::vector<ShellBatchSectionState>& plastic,
      const std::vector<sections::ShellLayeredLaw1History>& elastic) {
    count_ = plastic.size();
    plastic_.resize(count_);
    elastic_.resize(count_);
    output_.resize(count_);
    for (unsigned slab = 0; slab < 2; ++slab) {
      header_.plastic.section[slab] = plastic.data();
      header_.elastic_section[slab] = elastic.data();
    }
  }
  SetupReport Read(unsigned,std::size_t,cudaStream_t,const ShellBatchPlasticityBinding&,
      const ShellBatchOnePointSectionState* = nullptr) noexcept;
};
} // namespace qeph_activity_test::frozen_mixed
