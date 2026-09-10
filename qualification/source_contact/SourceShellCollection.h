#pragma once
#include "SourcePartContactFixture.h"
#include "lib_src/elements/ShellBatchBinding.h"

namespace crash::qualification::source_contact {
// Case-input adapter for the named original-source elastic experiment. Native
// family order is preserved, and each family entry maps back to its original
// source-parent slot. Every original node/parent remains present. This object
// does not impose source attachments, construct structural mass, allocate an
// owner or qualify a timestep. ShellBatchBinding owns native startup/mass.
class SourceShellCollection {
  public:
    FixtureReport Initialize(const SourcePartContactFixture&);
    bool prepared() const noexcept { return prepared_; }
    tl::fea::ShellBatchCollectionInput input() const noexcept {
        return prepared_ ? tl::fea::ShellBatchCollectionInput{qeph_.data(), t3_.data(), Q4Count, T3Count, NodeCount}
                         : tl::fea::ShellBatchCollectionInput{};
    }
    const auto& qeph_source_parents() const noexcept { return qeph_parent_; }
    const auto& t3_source_parents() const noexcept { return t3_parent_; }
  private:
    std::array<tl::fea::ShellQephBindingInput, Q4Count> qeph_{};
    std::array<tl::fea::ShellT3BindingInput, T3Count> t3_{};
    std::array<std::size_t, Q4Count> qeph_parent_{};
    std::array<std::size_t, T3Count> t3_parent_{};
    bool prepared_ = false;
};
}  // namespace crash::qualification::source_contact
