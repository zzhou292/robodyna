#include "CsvLedgerSegments.h"

#include <algorithm>

namespace crash::output {
CsvLedgerWriter::CsvLedgerWriter(const std::filesystem::path& directory,const std::string& header,
                                 const CsvLedgerPlan& plan,std::size_t cap)
    :directory_(directory),header_(header),plan_(plan) {
    ValidateCsvLedgerPlan(plan_,header_,cap);
    Require(std::filesystem::is_directory(directory_),"CSV ledger output directory is unavailable");
    // Future rollover destinations are also create-only. Recheck each at Open
    // to diagnose a conflict introduced after this externally serialized check.
    for(const auto& segment:plan_.segments)
        Require(!std::filesystem::exists(std::filesystem::symlink_status(directory_/segment.file)),"CSV ledger output already exists");
    try {Open(0);} catch(...) {Abort();throw;}
}
CsvLedgerWriter::~CsvLedgerWriter() {Abort();}
void CsvLedgerWriter::Open(std::size_t index) {
    Require(index<plan_.segments.size()&&!stream_.is_open(),"Invalid CSV segment transition");
    const auto path=directory_/plan_.segments[index].file;
    Require(!std::filesystem::exists(std::filesystem::symlink_status(path)),"CSV segment output already exists");
    stream_.clear();stream_.open(path,std::ios::binary);
    Require(bool(stream_),"Could not create CSV segment");
    stream_<<header_;Require(bool(stream_),"Could not write CSV segment header");
    segment_=index;bytes_=header_.size();
}
void CsvLedgerWriter::CloseChecked() {
    stream_.flush();Require(bool(stream_),"CSV segment flush failed");
    stream_.close();Require(!stream_.fail(),"CSV segment close failed");
}
void CsvLedgerWriter::CheckRow(std::uint64_t epoch,const std::string& row) const {
    Require(!failed_&&!finished_&&epoch==rows_+1&&epoch<=plan_.interval_count,"CSV row is not the next planned accepted interval");
    Require(!row.empty()&&row.size()<=plan_.max_row_bytes&&row.back()=='\n',"CSV row exceeds its encoded capacity");
    std::size_t columns=1,length=0;
    for(std::size_t i=0;i+1<row.size();++i) {
        const unsigned char c=row[i];
        Require(c>=32&&c<=126&&c!='"',"CSV row contains unsupported quoting or control bytes");
        if(c==',') {Require(length,"CSV row contains an empty field");length=0;++columns;}
        else ++length;
    }
    Require(length&&columns==plan_.column_count,"CSV row column count differs from its header");
    const bool rollover=epoch>plan_.segments[segment_].last_epoch;
    const auto next=segment_+(rollover?1:0);
    Require(next<plan_.segments.size(),"CSV row exceeds the segment plan");
    const auto bytes=rollover?header_.size():bytes_;
    Require(bytes<=plan_.segments[next].byte_cap&&row.size()<=plan_.segments[next].byte_cap-bytes,"CSV row exceeds segment byte capacity");
}
void CsvLedgerWriter::Append(std::uint64_t epoch,const std::string& row) {
    CheckRow(epoch,row);
    try {
        if(epoch>plan_.segments[segment_].last_epoch) {CloseChecked();Open(segment_+1);}
        stream_<<row;Require(bool(stream_),"CSV segment row write failed");
        bytes_+=row.size();rows_=epoch;
    } catch(...) {Abort();throw;}
}
void CsvLedgerWriter::Flush() {
    Require(!failed_&&!finished_,"CSV ledger is closed");
    try {stream_.flush();Require(bool(stream_),"CSV ledger flush failed");}
    catch(...) {Abort();throw;}
}
void CsvLedgerWriter::Finish() {
    Require(!failed_&&!finished_&&rows_==plan_.interval_count,"CSV ledger has not written its complete planned horizon");
    try {CloseChecked();finished_=true;} catch(...) {Abort();throw;}
}
void CsvLedgerWriter::Abort() noexcept {
    if(finished_)return;
    failed_=true;
    try {if(stream_.is_open())stream_.close();} catch(...) {}
}
} // namespace crash::output
