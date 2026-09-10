#pragma once
#include "chrono/SurfaceBinding.h"
#include "modelio/source_assembly/SourceAssembly.h"
#include <memory>

namespace crash::output::assembly {
namespace source = modelio::assembly;
struct SurfaceLimits {
    std::size_t max_nodes=2048, max_parents=1024, max_host_bytes=1024*1024;
};
struct ParentMapping {
    std::uint64_t element=0, part=0, material=0, section=0, curve=0;
    std::size_t source_index=0, family_index=0, first_triangle=0, triangle_count=0;
    unsigned source_elform=0;
    source::ShellFamily family=source::ShellFamily::Qeph;
};
// Immutable display association prepared directly from authenticated source.
// Every source node is present once in its TL global order. Every Q4 has two
// display triangles; T3 has one. No shell reference/mass construction, geometry
// scaling, nodal ownership or temporal authority. The complete source survives
// caller destruction. Limits cover derived payload, not the shared source bytes.
class SourceAssemblySurface {
  public:
    static SourceAssemblySurface Prepare(const source::SourceAssembly&, visual::Identity,
        std::uint64_t asset, std::uint64_t instance, SurfaceLimits = {});
    const source::SourceAssembly& source() const noexcept;
    const visual::Binding& binding() const noexcept;
    const std::vector<ParentMapping>& parents() const noexcept;
    const std::vector<std::uint64_t>& triangle_parents() const noexcept;
    std::size_t host_bytes() const noexcept;
  private:
    struct Impl;
    explicit SourceAssemblySurface(std::shared_ptr<const Impl> impl):impl_(std::move(impl)) {}
    std::shared_ptr<const Impl> impl_;
};
} // namespace crash::output::assembly
