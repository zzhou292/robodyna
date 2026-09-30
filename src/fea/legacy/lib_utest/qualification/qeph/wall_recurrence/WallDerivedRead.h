#pragma once
#include "WallBoostComparison.h"
#include "WallJobReport.h"

namespace tl::qualification::qeph::wall_recurrence {
struct WallDerivedReadBinding {
  std::string analysis_index_sha256,analysis_provenance_sha256;
  std::size_t remaining_set_bytes=0;
};
struct WallDerivedReadResult {
  WallJobSummary summary;
  WallDerivedReceipt receipt;
  bool summary_available=false;
  std::string diagnostic;
};
// The raw result must already have been authenticated with ReadRawJob against
// external hashes. The analysis producer/source trust is also external; an
// index or provenance cannot authenticate itself. This reader rechecks exact
// inventories, model/operator recipes and cheap numerical predicates. Saved
// spectra and decomposition residuals remain reviewed-producer measurements;
// no native map, Schur decomposition or Gram solve is performed here.
// A valid closed partial archive returns its receipt but no selectable summary.
// Complete failed numerical points retain per-step failures, including 4H0.
// Malformed archives leave the whole caller output unchanged.
bool ReadWallJobSummary(const std::filesystem::path&,const RawReadResult&,
                       const WallDerivedReadBinding&,WallDerivedReadResult&,std::string& error);
} // namespace tl::qualification::qeph::wall_recurrence
