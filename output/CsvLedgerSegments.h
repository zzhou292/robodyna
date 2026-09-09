#pragma once

#include "ArtifactInventory.h"
#include <fstream>
#include <vector>

namespace crash::output {

inline constexpr const char* kCsvLedgerSegmentsField="interval_ledger_segments";
inline constexpr std::size_t kCsvLedgerSegmentCap=8,kCsvLedgerCountCap=8;
inline constexpr std::size_t kCsvLedgerHeaderCap=8192,kCsvLedgerColumnCap=64;
inline constexpr std::size_t kCsvLedgerRowCap=64*26;
struct CsvLedgerSegment {
    std::string file;
    std::uint64_t first_epoch=0,last_epoch=0,row_count=0;
    std::size_t byte_cap=0; // Header plus row_count * maximum encoded row bytes.
};
struct CsvLedgerPlan {
    std::string logical_file,header_sha256;
    std::size_t column_count=0,max_row_bytes=0,total_bytes=0;
    std::uint64_t interval_count=0;
    std::vector<CsvLedgerSegment> segments;
};

// Flat unquoted CSV only. Deterministic maximum-row chunks, with a repeated
// exact header in each segment. First file retains logical_file; later files
// append -0001.csv, etc. Limits may be reduced for a smaller caller domain,
// never raised above the common artifact cap. Planning performs no file I/O.
CsvLedgerPlan PlanCsvLedger(const std::string& logical_file,const std::string& header,
                           std::uint64_t intervals,std::size_t max_row_bytes,
                           std::size_t file_cap=kArtifactFileCap);
// Exact metadata comparison, including deterministic names/ranges/capacities.
bool SameCsvLedgerPlan(const CsvLedgerPlan&,const CsvLedgerPlan&) noexcept;
// Recompute using actual authenticated header bytes; malformed or modified
// metadata throws. Readers additionally check actual row identities/counts.
void ValidateCsvLedgerPlan(const CsvLedgerPlan&,const std::string& actual_header,
                           std::size_t file_cap=kArtifactFileCap);

// The JSON value is an array of bounded plans. Required members reject
// duplicates; unknown extension members are ignored. Parse stages its return
// value and validates basic ranges, unique safe names, and declared byte caps;
// actual-header validation above completes the deterministic capacity proof.
std::vector<CsvLedgerPlan> ParseCsvLedgerSegments(const Value&);
void AppendCsvLedgerSegments(Document&,const CsvLedgerPlan* plans,std::size_t count);

// Owns only a serial output stream; no case, state, clock, or physical law.
// Caller preflights every contributor row before appending any of them.
// Files must be absent. Write/flush/close failure closes and poisons this
// writer; incomplete files remain evidence and are never overwritten.
class CsvLedgerWriter {
 public:
    CsvLedgerWriter(const std::filesystem::path& directory,const std::string& header,
                    const CsvLedgerPlan&,std::size_t file_cap=kArtifactFileCap);
    ~CsvLedgerWriter();
    CsvLedgerWriter(const CsvLedgerWriter&)=delete;
    CsvLedgerWriter& operator=(const CsvLedgerWriter&)=delete;
    void CheckRow(std::uint64_t accepted_epoch,const std::string& row) const;
    void Append(std::uint64_t accepted_epoch,const std::string& row);
    void Flush();
    void Finish(); // All planned rows required; checked close.
    void Abort() noexcept;
    std::uint64_t rows_written() const noexcept { return rows_; }
    bool failed() const noexcept { return failed_; }
 private:
    void Open(std::size_t segment);
    void CloseChecked();
    std::filesystem::path directory_;
    std::string header_;
    CsvLedgerPlan plan_;
    std::ofstream stream_;
    std::uint64_t rows_=0;
    std::size_t segment_=0,bytes_=0;
    bool failed_=false,finished_=false;
};

} // namespace crash::output
