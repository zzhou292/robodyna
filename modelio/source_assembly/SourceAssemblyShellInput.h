#pragma once
#include "SourceAssembly.h"
#include "lib_src/elements/ShellBatchBinding.h"

namespace crash::modelio::assembly {
// Owns declarations only. Copies/moves rebind the returned borrowed ranges on
// each input() call. Native startup, mass construction and runtime admission
// remain TL responsibilities and have not occurred when this adapter exists.
class SourceAssemblyShellInput {
  public:
    explicit SourceAssemblyShellInput(const SourceAssembly&);
    SourceAssemblyShellInput(const SourceAssemblyShellInput&) = default;
    SourceAssemblyShellInput(SourceAssemblyShellInput&&) = default;
    SourceAssemblyShellInput& operator=(const SourceAssemblyShellInput&) = delete;
    SourceAssemblyShellInput& operator=(SourceAssemblyShellInput&&) = delete;
    tl::fea::ShellBatchCollectionInput input() const noexcept;
    const auto& qeph_source_parents() const noexcept { return qeph_parents_; }
    const auto& t3_source_parents() const noexcept { return t3_parents_; }
    const SourceAssembly& source() const noexcept { return source_; }
  private:
    SourceAssembly source_;
    std::vector<tl::fea::ShellQephBindingInput> qeph_;
    std::vector<tl::fea::ShellT3BindingInput> t3_;
    std::vector<std::size_t> qeph_parents_, t3_parents_;
    std::size_t node_count_ = 0;
};
}  // namespace crash::modelio::assembly
