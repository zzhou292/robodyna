#pragma once
#include "ElasticCouponCase.h"
#include <memory>
#include <string>

namespace crash::case_data {
// Accepted-only result writer. A completed manifest requires every interval,
// initial/final frames and the full admitted horizon. Files are create-only;
// output failure never changes the solver's accepted state.
class ElasticCouponArtifacts {
  public:
    ElasticCouponArtifacts(const std::string& new_directory, const ElasticCouponCase&, unsigned frame_every);
    ~ElasticCouponArtifacts();
    void RecordInterval(const tl::fea::NodalStamp& base, const ElasticCouponMetrics& accepted);
    void WriteFrame(ElasticCouponCase&);
    void Finish(const ElasticCouponCase&, double elapsed_seconds);
    void Fail(const std::string& diagnostic) noexcept;
  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
}  // namespace crash::case_data
