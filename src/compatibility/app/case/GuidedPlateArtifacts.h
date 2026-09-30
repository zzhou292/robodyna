#pragma once

#include "GuidedPlateCase.h"
#include <memory>

namespace crash::case_data {
// Forecast and source/owner validation complete before creating the directory.
// Three bounded contiguous interval ledgers, immutable original wall, accepted
// mesh/fields and initial/final frames are required before completion. No output
// operation advances mechanics. All errors throw; call Fail after write errors.
class GuidedPlateArtifacts {
  public:
    GuidedPlateArtifacts(const std::string& new_directory,const std::string& verified_canonical_bytes,
                         const CanonicalWall&,const GuidedPlateCase&,unsigned frame_every);
    ~GuidedPlateArtifacts();
    GuidedPlateArtifacts(const GuidedPlateArtifacts&)=delete;
    GuidedPlateArtifacts& operator=(const GuidedPlateArtifacts&)=delete;
    void RecordInterval(const tl::fea::NodalStamp& base,const GuidedPlateMetrics& accepted);
    void WriteFrame(GuidedPlateCase&);
    void Finish(const GuidedPlateCase&,double elapsed_seconds);
    void Fail(const std::string& diagnostic) noexcept;
  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
} // namespace crash::case_data
