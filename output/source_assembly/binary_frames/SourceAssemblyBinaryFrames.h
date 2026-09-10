#pragma once
#include "output/full_shell/FullShellVisualizationRecords.h"
#include "output/source_assembly/SourceAssemblyWallFields.h"
#include <memory>

namespace crash::output::assembly::binary {
namespace records=full_shell;
struct Limits {
    AcceptedOutputLimits capture;
    records::RecordLimits records{2048,1024,3072,1024*1024,{}};
    // Exact two x/PLA buffers; excludes the separately capped accepted capture,
    // immutable Context, shared source, and per-file serialization buffers.
    std::size_t frame_bytes=256*1024;
};
// Component-only live accepted producer. It retains the existing qualified
// capture, not an FE owner/clock. Calls must serialize with stepping/readers.
// Capture reads exactly once through run.CaptureAccepted, validates the common
// phase, and selects preallocated records only after all checks succeed.
// Failed readbacks/trials never replace frame(). A rejected trial can still
// leave a newer, previously uncaptured accepted prefix available to Capture.
// No run manifest, completion receipt or full-vehicle replay admission is owned.
class SourceAssemblyBinaryFrames {
  public:
    SourceAssemblyBinaryFrames(const dynamics::SourceAssemblyWallCase&,visual::Identity,
                               std::uint64_t asset,Limits={});
    ~SourceAssemblyBinaryFrames();
    SourceAssemblyBinaryFrames(const SourceAssemblyBinaryFrames&)=delete;
    SourceAssemblyBinaryFrames& operator=(const SourceAssemblyBinaryFrames&)=delete;
    void Capture(dynamics::SourceAssemblyWallCase&);
    const records::Context& context() const noexcept;
    const SourceAssemblySurface& mapping() const noexcept;
    const records::FrameRecord* frame() const noexcept;
    std::size_t frame_bytes() const noexcept;
    // Per-frame files only. Uses unchanged create-only WriteFrame; I/O failure
    // may leave incomplete evidence, and never advances/changes accepted state.
    records::RecordFile Write(const std::filesystem::path&,const std::string& stem) const;
  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
    friend struct SourceAssemblyBinaryFrameTestAccess;
};
} // namespace crash::output::assembly::binary
