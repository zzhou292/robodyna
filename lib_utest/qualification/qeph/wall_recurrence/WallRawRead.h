#pragma once
#include "WallRawReport.h"

namespace tl::qualification::qeph::wall_recurrence {
struct RawReadBinding {
  unsigned cells=0;
  double normal_velocity=0;
  std::string provenance_sha256,index_sha256;
  std::size_t remaining_screen_byte_budget=0;
};
struct RawReadResult {
  RawJob job;
  RawJobReceipt receipt;
};
// Authenticate one closed artifact inventory against externally supplied
// hashes/identity. A closed inventory may retain an incomplete or numerically
// rejected collection for diagnosis; it never constitutes screen admission.
// No native maps are evaluated. Source-file authenticity is an external duty.
// Missing final index, malformed records, excess bytes or hash/identity failure
// preserve the complete caller output. Stored numerical verdicts are evidence,
// not trusted decisions; later analysis must use the retained physical samples.
bool ReadRawJob(const std::filesystem::path&,const RawReadBinding&,
                RawReadResult&,std::string& error);
} // namespace tl::qualification::qeph::wall_recurrence
