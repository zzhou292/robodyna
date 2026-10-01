#pragma once
#include "CaptureSession.h"
#include <sstream>

namespace robodyna::examples {
struct CaptureSession::Impl {
    explicit Impl(CaptureOptions value) : options(std::move(value)) {}
    CaptureOptions options;
    std::ostringstream index;
    std::uint64_t step = 0;
    double time = 0;
    std::size_t frames = 0, png_bytes = 0;
    unsigned width = 0, height = 0;
    bool observed = false, configured = false, finished = false;
    CaptureVisual* visual = nullptr;
    void CheckEndpoint(chrono::ChSystem&) const;
    crash::output::Document Summary(chrono::ChSystem&, const crash::output::Document&) const;
};
}  // namespace robodyna::examples
