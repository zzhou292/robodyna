#pragma once
#include "SourceAssembly.h"
#include "Law1ExecutionPolicy.h"
#include "lib_src/elements/ShellBatchPlasticityBinding.h"

namespace crash::modelio::assembly {
enum class MaterialRatePolicy { OpenRadiossDirectImportDefault };
class SourceAssemblyShellInput;
// Mandatory named policy: original LAW44 C/P/VP declarations stay in SourceAssembly;
// LAW1 has no rate declarations and uses canonical unused native controls.
// No default argument silently enables/disables filtering. Curves borrow from
// shared immutable source ownership, so copies/moves never dangle curve pointers.
class SourceAssemblyMaterialInput {
  public:
    SourceAssemblyMaterialInput(const SourceAssembly&, MaterialRatePolicy);
    // Explicit source metric comes from the existing reference-input adapter.
    SourceAssemblyMaterialInput(const SourceAssemblyShellInput&,MaterialRatePolicy,Law1ExecutionProfile);
    SourceAssemblyMaterialInput(const SourceAssemblyMaterialInput&) = default;
    SourceAssemblyMaterialInput(SourceAssemblyMaterialInput&&) = default;
    SourceAssemblyMaterialInput& operator=(const SourceAssemblyMaterialInput&) = delete;
    SourceAssemblyMaterialInput& operator=(SourceAssemblyMaterialInput&&) = delete;
    tl::fea::ShellBatchPlasticityBindingInput input() const noexcept;
    MaterialRatePolicy rate_policy() const noexcept { return policy_; }
    const Law1ExecutionPolicy& execution_policy() const noexcept { return execution_; }
    const SourceAssembly& source() const noexcept { return source_; }
  private:
    SourceAssembly source_;
    MaterialRatePolicy policy_;
    Law1ExecutionPolicy execution_;
    void Pack();
    std::vector<tl::fea::ShellPlasticityCurveInput> curves_;
    std::vector<tl::fea::ShellPlasticityMaterialInput> materials_;
    std::vector<tl::fea::ShellPlasticitySectionInput> sections_;
    std::vector<tl::fea::ShellPlasticityParentInput> parents_;
};
}  // namespace crash::modelio::assembly
