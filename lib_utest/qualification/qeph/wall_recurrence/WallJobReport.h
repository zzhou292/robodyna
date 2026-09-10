#pragma once
#include "WallJobAnalysis.h"
#include "WallRawRead.h"

namespace tl::qualification::qeph::wall_recurrence {
struct WallDerivedReceipt {
  unsigned cells=0;
  double normal_velocity=0;
  std::string raw_index_sha256,raw_provenance_sha256,analysis_provenance_sha256;
  std::vector<RawFileReceipt> files;
  std::size_t total_bytes=0;
  bool final_index_present=false,analysis_complete=false,analysis_passed=false;
};
// The authenticated raw result must outlive this writer and remain immutable.
// It is not re-read or re-evaluated. External source authentication is still the
// launcher's responsibility; this writer binds exact supplied artifact hashes.
// Each callback publishes create-only evidence, then a new progress inventory.
// No boost comparison, timestep selection or trajectory admission occurs here.
class WallJobReportWriter {
 public:
  WallJobReportWriter(const std::filesystem::path& new_directory,const RawReadResult&,
    const RawReadBinding&,const std::string& analysis_provenance_bytes,
    const std::string& analysis_provenance_origin,std::size_t remaining_set_bytes);
  WallJobReportWriter(const WallJobReportWriter&)=delete;
  WallJobReportWriter& operator=(const WallJobReportWriter&)=delete;
  WallJobReportWriter(WallJobReportWriter&&)=delete;
  WallJobReportWriter& operator=(WallJobReportWriter&&)=delete;
  void operator()(const WallJobAnalysis&,WallJobProgress);
  const WallDerivedReceipt& receipt() const { return receipt_; }
 private:
  const RawReadResult& raw_;
  RawReadBinding binding_;
  std::filesystem::path directory_;
  std::string provenance_origin_;
  std::size_t budget_=0;
  WallDerivedReceipt receipt_;
  unsigned sequence_=0;
  std::array<std::array<bool,3>,6> amplitude_seen_{},amplitude_complete_{},amplitude_passed_{};
  std::array<std::array<bool,2>,6> contact_seen_{},contact_complete_{},contact_passed_{};
  std::array<bool,6> step_seen_{},step_complete_{},step_passed_{};
  std::array<std::string,6> context_hash_{},context_data_hash_{};
  RawFileReceipt Write(const std::string&,const std::string&);
  crash::output::Document Header(const char* kind) const;
  void Context(const WallJobAnalysis&,unsigned step,unsigned amplitude);
  void Inventory(const WallJobAnalysis*,bool final);
  void ValidateCaptured(const WallJobAnalysis&) const;
  std::string RawHash(const std::string&) const;
  void RawLink(crash::output::Document&,const char* label,const std::string&) const;
};
// Canonical matrix descriptor hash covers exact compact raw matrix JSON.
// Reconstruct operators using the named owning B and D routines, then compare
// this hash. Failed nonfinite matrices retain marker bits in their hash input.
std::string WallDerivedMatrixHash(const Eigen::MatrixXd&);
} // namespace tl::qualification::qeph::wall_recurrence
