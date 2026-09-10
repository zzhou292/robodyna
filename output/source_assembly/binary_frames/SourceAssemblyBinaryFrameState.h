#pragma once
#include "SourceAssemblyBinaryFrames.h"
#include <array>
#include <optional>

namespace crash::output::assembly::binary {
struct SourceAssemblyBinaryFrames::Impl {
    SourceAssemblyAcceptedOutput capture;
    std::optional<records::Context> context;
    std::array<records::FrameRecord,2> frames;
    std::size_t bytes=0;
    unsigned published=0;
    bool available=false;
};
} // namespace crash::output::assembly::binary
