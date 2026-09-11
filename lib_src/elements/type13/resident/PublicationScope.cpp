// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Storage.h"

namespace tl::fea::type13 {
BatchReport Batch::PreflightAttach(const NodalStamp& stamp,
    const Type13NodeContributions& source, std::uint64_t configuration,
    std::uint64_t qualification, const ShellBatchStartup& startup,
    BatchAssembly assembly, const ShellBatchPublication* claimant) const noexcept {
  if (!impl_) return {BatchStatus::NotInitialized, "TYPE13 batch is not initialized"};
  const auto& state = *impl_;
  if (!state.usable) return {BatchStatus::Unusable, "TYPE13 CUDA storage is poisoned"};
  if (!state.bound) return {BatchStatus::NotBound, "TYPE13 initial live sources are not bound"};
  if (!claimant || state.publication_scope || state.pending || state.accepted_stamp.epoch ||
      !trial_identity::SameStamp(stamp, state.accepted_stamp) ||
      !state.source.Matches(source) || configuration != state.config.configuration_id ||
      qualification != state.config.qualification_id || assembly != state.config.assembly ||
      !shell_startup_detail::SameStartup(startup, state.config.startup)) {
    return {BatchStatus::InvalidInput, "TYPE13 publication requires the same bound initial scope"};
  }
  return {};
}
void Batch::AttachPublication(const ShellBatchPublication* claimant) noexcept {
  impl_->publication_scope = claimant;
}
void Batch::ReleasePublication(const ShellBatchPublication* claimant) noexcept {
  if (impl_ && impl_->publication_scope == claimant) impl_->publication_scope = nullptr;
}
void Batch::Poison() noexcept {
  if (impl_) {
    impl_->usable = false;
    impl_->Discard();
  }
}
} // namespace tl::fea::type13
