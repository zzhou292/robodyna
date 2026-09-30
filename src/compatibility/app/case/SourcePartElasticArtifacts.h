#pragma once
#include "source_part_elastic/SourcePartElasticCase.h"
#include <memory>
#include <string>

namespace crash::cases::source_part_elastic {
// Create-only accepted geometry/phase archive. The writer owns presentation
// bindings, never a clock, mechanics, trial or restart history. Completion
// requires every committed interval and initial/final frames. Sparse frames
// retain the exact preceding base/kick times, including the first half kick.
class SourcePartElasticArtifacts {
  public:
    SourcePartElasticArtifacts(const std::string& new_directory, SourcePartElasticCase&,
        std::uint64_t required_steps, unsigned frame_every, std::uint64_t run_id, std::uint64_t topology_id);
    ~SourcePartElasticArtifacts();
    SourcePartElasticArtifacts(const SourcePartElasticArtifacts&) = delete;
    SourcePartElasticArtifacts& operator=(const SourcePartElasticArtifacts&) = delete;
    void RecordInterval(const tl::fea::NodalStamp& base, const tl::fea::NodalStamp& accepted, const Diagnostics&);
    void WriteFrame(SourcePartElasticCase&);
    void Finish(SourcePartElasticCase&, double elapsed_seconds);
    void Fail(const std::string&) noexcept;
  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
} // namespace crash::cases::source_part_elastic
