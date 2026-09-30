#include "AcceptedReplayData.h"
#include "CsvLedgerSegments.h"

#include <array>
#include <charconv>
#include <cmath>
#include <set>
#include <string_view>

namespace crash::output::replay_detail {
namespace {
constexpr std::array<const char*,3> Logical{{"accepted-intervals.csv","shell-intervals.csv","contact-intervals.csv"}};
constexpr std::array<std::string_view,6> Identity{{"owner_id","base_epoch","attempt","force_eval_time_s",
                                               "accepted_epoch","accepted_time_s"}};
struct Row {
    std::uint64_t owner=0,base=0,attempt=0,epoch=0;
    double base_time=0,time=0;
};
template<class T> T Number(std::string_view value) {
    T result{};
    const auto parsed=std::from_chars(value.data(),value.data()+value.size(),result);
    Require(parsed.ec==std::errc{}&&parsed.ptr==value.data()+value.size(),"Invalid guided ledger numeric field");
    return result;
}
std::size_t Columns(std::string_view line,std::array<std::string_view,kCsvLedgerColumnCap>& fields) {
    std::size_t count=0,start=0;
    while(true) {
        const auto end=line.find(',',start);
        Require(count<fields.size(),"Guided ledger column cap exceeded");
        fields[count++]=line.substr(start,end==line.npos?line.size()-start:end-start);
        Require(!fields[count-1].empty(),"Empty guided ledger column");
        if(end==line.npos) return count;
        start=end+1;
    }
}
bool Same(const Row& a,const Row& b) {
    return a.owner==b.owner&&a.base==b.base&&a.attempt==b.attempt&&a.epoch==b.epoch&&
           Bits(a.base_time)==Bits(b.base_time)&&Bits(a.time)==Bits(b.time);
}

// Retain only one authenticated segment per contributor. At most three 32MiB
// strings are live (plus the next segment during replacement), not N rows or
// the complete archive. Views never outlive the corresponding segment bytes.
class Cursor {
 public:
    Cursor(const Bundle& bundle,const CsvLedgerPlan& plan):bundle_(bundle),plan_(plan) { Open(0); }
    Row Next() {
        if(segment_rows_==plan_.segments[segment_].row_count) {
            Require(position_==bytes_.size(),"Guided ledger segment contains extra rows");
            Require(segment_+1<plan_.segments.size(),"Guided ledger ended before the declared horizon");
            Open(segment_+1);
        }
        const auto line=Line(plan_.max_row_bytes);
        std::array<std::string_view,kCsvLedgerColumnCap> fields;
        Require(Columns(line,fields)==plan_.column_count,"Guided ledger row has a changed column count");
        Row result;
        result.owner=Number<std::uint64_t>(fields[0]); result.base=Number<std::uint64_t>(fields[1]);
        result.attempt=Number<std::uint64_t>(fields[2]); result.base_time=Number<double>(fields[3]);
        result.epoch=Number<std::uint64_t>(fields[4]); result.time=Number<double>(fields[5]);
        Require(std::isfinite(result.base_time)&&std::isfinite(result.time),"Guided ledger time is nonfinite");
        for(std::size_t c=6;c<plan_.column_count;++c)
            Require(std::isfinite(Number<double>(fields[c])),"Guided ledger diagnostic is nonfinite");
        const auto& segment=plan_.segments[segment_];
        Require(result.epoch==segment.first_epoch+segment_rows_&&result.epoch<=segment.last_epoch,
                "Guided ledger actual epoch differs from segment metadata");
        ++segment_rows_;
        return result;
    }
    void Finish() const {
        Require(segment_+1==plan_.segments.size()&&segment_rows_==plan_.segments[segment_].row_count&&
                position_==bytes_.size(),"Guided ledger completion/row count differs from its plan");
    }
 private:
    std::string_view Line(std::size_t limit) {
        const auto end=bytes_.find('\n',position_);
        Require(end!=std::string::npos&&end-position_+1<=limit,"Guided ledger line is incomplete or exceeds its cap");
        const std::string_view result(bytes_.data()+position_,end-position_);
        position_=end+1;
        return result;
    }
    void Open(std::size_t index) {
        segment_=index; segment_rows_=0; position_=0;
        bytes_=VerifiedBytes(bundle_,plan_.segments[index].file);
        Require(bytes_.size()<=plan_.segments[index].byte_cap,"Guided segment exceeds its declared byte capacity");
        const auto line=Line(kCsvLedgerHeaderCap);
        const std::string header=std::string(line)+'\n';
        if(index==0) {
            ValidateCsvLedgerPlan(plan_,header); // Reuses deterministic names/ranges/capacities, never relaxes file cap.
            Require(plan_.column_count>=Identity.size(),"Guided ledger lacks its six identity columns");
            std::array<std::string_view,kCsvLedgerColumnCap> fields;
            Require(Columns(line,fields)==plan_.column_count,"Guided ledger header count changed");
            for(std::size_t i=0;i<Identity.size();++i)
                Require(fields[i]==Identity[i],"Guided ledger identity header changed");
            header_=header;
        } else Require(header==header_,"Guided ledger segment repeats a different header");
    }
    const Bundle& bundle_;
    const CsvLedgerPlan& plan_;
    std::string bytes_,header_;
    std::size_t segment_=0,position_=0;
    std::uint64_t segment_rows_=0;
};
} // namespace

void CheckGuidedLedgers(Bundle& bundle,const Document& config,const Document& manifest) {
    const bool split=bundle.info.schema=="robo_dyna.guided_plate_artifacts.v2";
    const bool config_has=config.HasMember(kCsvLedgerSegmentsField),manifest_has=manifest.HasMember(kCsvLedgerSegmentsField);
    if(!split) {
        Require(!config_has&&!manifest_has,"Legacy guided v1 must use its single-file ledger protocol");
        return; // Preserve the prior v1 admission; do not reinterpret historical CSV contents.
    }
    Require(config_has&&manifest_has,"Guided v2 requires matching configuration/manifest segment metadata");
    const auto plans=ParseCsvLedgerSegments(Member(config,kCsvLedgerSegmentsField));
    const auto copies=ParseCsvLedgerSegments(Member(manifest,kCsvLedgerSegmentsField));
    Require(plans.size()==Logical.size()&&copies.size()==plans.size(),"Guided v2 requires exactly three logical ledgers");
    Require(bundle.info.final_epoch<=1000000,"Guided interval horizon exceeds the bounded experiment");
    bool any_split=false;
    std::set<std::string> files;
    for(std::size_t i=0;i<plans.size();++i) {
        Require(plans[i].logical_file==Logical[i]&&SameCsvLedgerPlan(plans[i],copies[i])&&
                plans[i].interval_count==bundle.info.final_epoch,"Guided segment plans disagree or change ledger identity/horizon");
        any_split=any_split||plans[i].segments.size()>1;
        for(const auto& segment:plans[i].segments)
            Require(files.insert(segment.file).second&&bundle.inventory.count(segment.file),
                    "Guided ledger segment is duplicate or missing from inventory");
    }
    Require(any_split,"Guided artifact v2 is reserved for an actually segmented archive");
    for(const auto& item:bundle.inventory) {
        const auto& name=item.first;
        if(name.size()>=4&&name.substr(name.size()-4)==".csv")
            Require(name=="accepted-frames.csv"||files.count(name),"Guided v2 contains an undeclared CSV ledger segment");
    }
    std::array<Cursor,3> cursor{{Cursor(bundle,plans[0]),Cursor(bundle,plans[1]),Cursor(bundle,plans[2])}};
    Row previous;
    std::size_t frame=1;
    for(std::uint64_t epoch=1;epoch<=bundle.info.final_epoch;++epoch) {
        const auto row=cursor[0].Next();
        Require(Same(row,cursor[1].Next())&&Same(row,cursor[2].Next()),"Guided contributor interval stamps disagree");
        Require(row.owner==bundle.info.owner_id&&row.epoch==epoch&&row.base==epoch-1&&row.attempt>previous.attempt&&
                Bits(row.base_time)==Bits(previous.time)&&Bits(row.base_time+bundle.fixed_dt)==Bits(row.time),
                "Guided interval owner/epoch/attempt or fixed-step time continuity changed");
        if(frame<bundle.entries.size()&&bundle.entries[frame].epoch==epoch) {
            auto& saved=bundle.entries[frame];
            Require(Bits(saved.time)==Bits(row.time),"Guided saved frame time differs from its actual interval");
            saved.interval_attempt=row.attempt; saved.interval_base_time=row.base_time; ++frame;
        }
        previous=row;
    }
    for(const auto& c:cursor) c.Finish();
    Require(frame==bundle.entries.size()&&Bits(previous.time)==Bits(bundle.info.final_time),
            "Guided ledger horizon does not cover every saved frame");
}

} // namespace crash::output::replay_detail
