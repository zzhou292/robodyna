#pragma once
#include "FrameGeometryState.h"

namespace crash::visual::full_shell {
struct FullShellFrameGeometry::Impl : detail::FrameGeometryState {
    Impl(const output::full_shell::source::PreparedSourceMapping& m, const output::full_shell::Context& c)
        : detail::FrameGeometryState(c), mapping(m) {}
    output::full_shell::source::PreparedSourceMapping mapping;
};
namespace detail {
void CheckContext(const output::full_shell::source::PreparedSourceMapping&, const output::full_shell::Context&);
} // namespace detail
} // namespace crash::visual::full_shell
