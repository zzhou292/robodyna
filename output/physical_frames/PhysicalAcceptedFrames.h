#pragma once
#include "Mapping.h"
#include "Buffers.h"
namespace crash::output::physical_frames {
// One accepted owner/publisher readback. Calls serialize with owner operations.
// Source mapping is immutable; frames contain original-order binary64 shell
// positions, actual native plastic values and independently typed activity.
class PhysicalAcceptedFrames {
  public:
    PhysicalAcceptedFrames(const Mapping&,Run&,records::Identity,Limits={});
    ~PhysicalAcceptedFrames();
    PhysicalAcceptedFrames(const PhysicalAcceptedFrames&)=delete;
    PhysicalAcceptedFrames& operator=(const PhysicalAcceptedFrames&)=delete;
    static Forecast Preflight(const Mapping&,const records::Context&,Limits={});
    void Capture(Run&);
    const Mapping& mapping() const noexcept;
    const records::Context& context() const noexcept;
    const Forecast& forecast() const noexcept;
    const records::FrameRecord* frame() const noexcept;
    const records::activity::ActivityRecord* activity() const noexcept;
  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
} // namespace crash::output::physical_frames
