#pragma once
#include "WallRawCapture.h"
#include "output/ArtifactIO.h"
#include <filesystem>

namespace tl::qualification::qeph::wall_recurrence {
constexpr std::size_t RawFileByteCap=32u*1024*1024,ScreenSetByteCap=96u*1024*1024,ProvenanceByteCap=64u*1024;
struct RawFileReceipt {
  std::string name,sha256;
  std::size_t bytes=0;
};
struct RawJobReceipt {
  unsigned cells=0; double normal_velocity=0;
  std::vector<RawFileReceipt> files;
  std::size_t total_bytes=0;
  bool final_index_present=false,collection_complete=false;
};
// Parse bounded exact source/build provenance JSON. Structural validity and
// its byte hash are checked here; root owns authentication of listed sources.
crash::output::Document ParseRawProvenance(const std::string& exact_bytes);
std::string EncodeRawJson(const crash::output::Document&);

// Create-only incremental artifact writer, externally serialized like
// ArtifactIO. Constructor writes exact provenance plus initial inventory.
// Every callback writes its payload once, then a small new inventory. Earlier
// files survive callback/resource failure; an interrupted job has no completed
// final index. No atomic/exclusive multi-file transaction or resume claim.
class RawJobWriter {
 public:
  RawJobWriter(const std::filesystem::path& new_directory,unsigned cells,double normal_velocity,
               const std::string& provenance_bytes,const std::string& provenance_origin,
               std::size_t remaining_screen_byte_budget);
  RawJobWriter(const RawJobWriter&)=delete;
  RawJobWriter& operator=(const RawJobWriter&)=delete;
  RawJobWriter(RawJobWriter&&)=delete;
  RawJobWriter& operator=(RawJobWriter&&)=delete;
  void operator()(const RawJob&,RawProgress);
  const RawJobReceipt& receipt() const { return receipt_; }
 private:
  std::filesystem::path directory_;
  std::size_t budget_=0;
  RawJobReceipt receipt_;
  std::string provenance_hash_,provenance_origin_,model_hash_;
  unsigned sequence_=0;
  bool model_seen_=false;
  std::array<std::array<bool,3>,6> native_seen_{};
  std::array<std::array<bool,2>,6> contact_seen_{};
  std::array<std::array<bool,3>,6> native_complete_{};
  std::array<std::array<unsigned,3>,6> native_columns_{};
  std::array<std::array<bool,2>,6> contact_raw_complete_{},contact_checks_complete_{},contact_checks_passed_{};
  RawFileReceipt Write(const std::string& name,const std::string& bytes);
  crash::output::Document Header(const char* kind) const;
  void Inventory(const RawJob* job,bool final);
};
// Checks the six unique (cells,boost) receipts and all retained byte totals.
// It does not authenticate external files or imply any numerical admission.
// The caller adds derived-report bytes before the shared 96 MiB cap check.
std::size_t RawSetBytes(const std::array<RawJobReceipt,6>&,std::size_t derived_report_bytes=0);
} // namespace tl::qualification::qeph::wall_recurrence
