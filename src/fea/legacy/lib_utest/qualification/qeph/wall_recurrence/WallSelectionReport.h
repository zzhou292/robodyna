#pragma once
#include "WallDerivedRead.h"
#include "WallScreenSelection.h"

namespace tl::qualification::qeph::wall_recurrence {
constexpr std::size_t WallSelectionReportByteCap=2u*1024*1024;
struct WallSelectionInput {
  unsigned cells=0;
  double normal_velocity=0;
  std::filesystem::path raw_directory,derived_directory;
  std::string raw_index_sha256,raw_provenance_sha256,derived_index_sha256,derived_provenance_sha256;
  std::size_t raw_bytes=0,derived_bytes=0;
};
// Canonical protocol order: (1,0),(1,-8),(1,+8),(2,0),(2,-8),(2,+8).
// This only checks the declared six bindings; readers authenticate actual files.
unsigned WallSelectionInputSlot(unsigned cells,double velocity);
std::size_t ValidateWallSelectionInputs(const std::array<WallSelectionInput,6>&);
struct WallSelectionReceipt {
  std::vector<RawFileReceipt> files;
  std::size_t total_bytes=0;
  bool final_index_present=false,decision_complete=false,passed=false;
  double selected_h=0;
};
// Root-pinned config/source trust remains external. Inputs are authenticated
// one job at a time by the owning readers before RecordJob. No raw data tree,
// dense operator or retained borrowed model is copied here. IO is create-only
// per file; callers stop on exceptions and account for any partial file.
class WallSelectionReportWriter {
 public:
  WallSelectionReportWriter(const std::filesystem::path& new_directory,
    const std::array<WallSelectionInput,6>&,const std::string& exact_provenance,
    const std::string& provenance_origin,std::size_t remaining_set_bytes);
  WallSelectionReportWriter(const WallSelectionReportWriter&)=delete;
  WallSelectionReportWriter& operator=(const WallSelectionReportWriter&)=delete;
  void RecordJob(unsigned slot,const RawReadResult&,const WallDerivedReadResult&);
  void RecordBoost(const WallBoostComparison&);
  void Finish(const WallScreenSelection&);
  const WallSelectionReceipt& receipt() const { return receipt_; }
 private:
  std::filesystem::path directory_;
  std::array<WallSelectionInput,6> inputs_;
  std::array<RawJobReceipt,6> raw_;
  std::array<WallDerivedReceipt,6> derived_;
  std::array<bool,6> jobs_seen_{},summaries_available_{};
  std::array<bool,4> boosts_seen_{},boosts_available_{};
  std::array<std::array<bool,6>,6> job_pass_{};
  std::array<std::array<bool,6>,4> boost_pass_{};
  std::string provenance_hash_,provenance_origin_;
  std::size_t budget_=0,input_bytes_=0;
  unsigned sequence_=0;
  WallSelectionReceipt receipt_;
  RawFileReceipt Write(const std::string&,const std::string&);
  crash::output::Document Header(const char*) const;
  void Inventory(bool final,const WallScreenSelection* selection=nullptr);
};
namespace selection_json {
crash::output::Document Input(const WallSelectionInput&);
crash::output::Document Boost(const WallBoostComparison&,const RawJobReceipt& zero_raw,
  const RawJobReceipt& boost_raw,const WallDerivedReceipt& zero_derived,const WallDerivedReceipt& boost_derived);
crash::output::Document Selection(const WallScreenSelection&);
}
} // namespace tl::qualification::qeph::wall_recurrence
