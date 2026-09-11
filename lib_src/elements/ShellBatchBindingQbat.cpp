// SPDX-License-Identifier: AGPL-3.0-or-later
#include "ShellBindingValues.h"

namespace tl::fea {
ShellBindingReport ShellBatchBinding::InitializeFormulations(
    const ShellFormulationCollectionInput& input,const ShellHostBindingLimits& limits) noexcept {
  using namespace shell_binding_detail;
  if(prepared_)
    return Error(ShellBindingStatus::AlreadyInitialized,"Shell binding is immutable after initialization");
  if(!input.qbat_count)
    return Error(ShellBindingStatus::InvalidInput,"Formulation collection requires an explicit QBAT family");
  return InitializeImpl(input.shells,false,limits,true,input.qbat,input.qbat_count);
}
const qbat::Reference& ShellBatchBinding::qbat_reference(std::size_t i) const noexcept {
  static const qbat::Reference empty;
  return prepared_&&i<data_.qbat_count?data_.qbat[i].reference:empty;
}
const std::array<std::size_t,4>& ShellBatchBinding::qbat_nodes(std::size_t i) const noexcept {
  static const std::array<std::size_t,4> empty{};
  return prepared_&&i<data_.qbat_count?data_.qbat[i].nodes:empty;
}
std::uint64_t ShellBatchBinding::qbat_source_id(std::size_t i) const noexcept {
  return prepared_&&i<data_.qbat_count?data_.qbat[i].source_id:0;
}
std::size_t ShellBatchBinding::startup_scratch_bytes() const noexcept {
  if(!prepared_) return 0;
  const bool legacy=data_.inventory.words()[0]==3;
  return shell_binding_detail::ScratchBytes(data_.qeph_count,data_.t3_count,
      data_.node_count,legacy,data_.qbat_count);
}
} // namespace tl::fea
