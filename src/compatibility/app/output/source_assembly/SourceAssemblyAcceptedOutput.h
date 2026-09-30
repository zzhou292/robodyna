#pragma once
#include "SourceAssemblySectionFields.h"
#include "case/source_assembly/SourceAssemblyBindings.h"
#include "chrono/NodalMeshOutput.h"
#include "lib_src/elements/ShellBatchPublication.h"
#include <memory>

namespace crash::output::assembly {
struct AcceptedOutputLimits {
    visual::NodalCaptureLimits nodal{2048,512*1024};
    SurfaceLimits surface{};
    // Section double buffering, native result staging and parent color values.
    // Nodal captures, shared source/bindings and Chrono mesh have separate caps.
    std::size_t max_section_host_bytes=4*1024*1024;
};
// Accepted output composition only. The existing owner/publication/batches are
// borrowed during each call; no mechanics/clock or per-step allocation lives
// here. The engine must supply its authenticated SourceAssemblyBindings and the
// corresponding joined plastic batches. Reference identity is checked for every
// read-back parent; material/source authority remains that startup composition.
// Publish stages both families, all three points and native thickness, validates
// source/reference/accepted stamps, then uses the existing accepted-only nodal
// output. Only infallible output-buffer selection follows mesh success. Calls,
// rendering and physics stepping must be externally serialized.
class SourceAssemblyAcceptedOutput {
  public:
    SourceAssemblyAcceptedOutput();
    ~SourceAssemblyAcceptedOutput();
    SourceAssemblyAcceptedOutput(const SourceAssemblyAcceptedOutput&)=delete;
    SourceAssemblyAcceptedOutput& operator=(const SourceAssemblyAcceptedOutput&)=delete;
    visual::Report Initialize(const tl::fea::FENodalState&,
        const cases::source_assembly::SourceAssemblyBindings&, visual::Identity,
        std::uint64_t asset, AcceptedOutputLimits = {});
    visual::Report Publish(tl::fea::FENodalState&, tl::fea::qeph::QephBatch&,
        tl::fea::t3::T3Batch&, const tl::fea::ShellBatchPublication&);
    const SourceAssemblySurface* mapping() const noexcept;
    const visual::NodalMeshOutput* nodal() const noexcept;
    SectionView qeph() const noexcept;
    SectionView t3() const noexcept;
    const std::vector<ReplayParentScalar>* parent_scalars() const noexcept;
    // Exact owner-engine diagnostics. No synthesized energy interpretation.
    const tl::fea::ShellBatchDiagnostics* diagnostics() const noexcept;
    std::size_t section_host_bytes() const noexcept;
    // Views become available only on success and expire on the next successful
    // Publish or destruction. They never refer to the rejected/trial slab.
  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
} // namespace crash::output::assembly
