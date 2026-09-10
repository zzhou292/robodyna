#include "AcceptedReplaySourcePartWall.h"
#include "SourcePartWallArtifactSchema.h"
#include "CsvLedgerSegments.h"
#include "AcceptedReplayCsv.h"
#include <charconv>
#include <sstream>
namespace crash::output::replay_detail {
void ReadSourcePartWallIntervals(Bundle& b,const Document& config,const Document& manifest) {
    const auto planned=ParseCsvLedgerSegments(Member(config,kCsvLedgerSegmentsField));
    const auto completed=ParseCsvLedgerSegments(Member(manifest,kCsvLedgerSegmentsField));
    Require(planned.size()==1&&completed.size()==1,"Wall replay needs one interval ledger");
    ValidateCsvLedgerPlan(planned[0],SourcePartWallIntervalHeader);ValidateCsvLedgerPlan(completed[0],SourcePartWallIntervalHeader);
    const auto& plan=completed[0];
    Require(planned[0].logical_file=="accepted-intervals.csv"&&planned[0].interval_count==Unsigned(config,"required_steps")&&
        plan.interval_count==b.info.final_epoch&&SameCsvLedgerPlan(plan,PlanCsvLedger(planned[0].logical_file,
        SourcePartWallIntervalHeader,b.info.final_epoch,planned[0].max_row_bytes)),"Wall ledger is not the declared accepted prefix");
    std::uint64_t previous=0,last_attempt=0;double previous_time=0;std::size_t frame=1;
    for(const auto& segment:plan.segments) {
        std::istringstream input(VerifiedBytes(b,segment.file));std::string line;
        Require(bool(std::getline(input,line))&&line+"\n"==SourcePartWallIntervalHeader,"Wall interval header mismatch");std::uint64_t count=0;
        while(std::getline(input,line)) {
            std::array<std::string,SourcePartWallIntervalColumns> columns;std::istringstream row(line);
            for(auto& text:columns)Require(bool(std::getline(row,text,','))&&!text.empty(),"Incomplete wall interval");Require(row.eof(),"Extra wall interval column");
            const auto owner=ReplayCsvNumber<std::uint64_t>(columns[0]),base=ReplayCsvNumber<std::uint64_t>(columns[1]);
            const auto attempt=ReplayCsvNumber<std::uint64_t>(columns[2]),epoch=ReplayCsvNumber<std::uint64_t>(columns[4]);
            const auto base_time=ReplayCsvNumber<double>(columns[3]),time=ReplayCsvNumber<double>(columns[5]);std::array<double,28> values{};
            for(unsigned i=6;i<columns.size();++i){values[i-6]=ReplayCsvNumber<double>(columns[i]);Require(std::isfinite(values[i-6]),"Nonfinite wall interval");}
            Require(owner==b.info.owner_id&&base==previous&&epoch==base+1&&attempt>last_attempt&&Bits(base_time)==Bits(previous_time)&&
                Bits(time)==Bits(base_time+b.fixed_dt)&&Bits(values[0])==Bits(base_time+.5*b.fixed_dt)&&
                Bits(values[1])==Bits(base==0?.5*b.fixed_dt:b.fixed_dt),"Wall interval owner, phase or attempt mismatch");
            Require(values[19]>=0&&values[19]<=b.wall_penetration_cap&&values[20]>=0&&values[20]<=117&&std::floor(values[20])==values[20],
                "Wall interval penetration or active-node count is invalid");
            Require(values[27]>=0&&values[27]<=117&&std::floor(values[27])==values[27],"Wall interval strict-separation count is invalid");
            Require(values[26]>=0&&Bits(values[22])==Bits(b.wall_energy_allowance)&&
                std::abs(values[4])+values[26]<=b.wall_energy_allowance,"Wall interval changed or exceeded the fixed physical energy budget");
            if(frame<b.entries.size()&&b.entries[frame].epoch==epoch) {
                auto& saved=b.entries[frame];Require(Bits(saved.time)==Bits(time),"Wall saved frame time differs from interval");
                saved.interval_attempt=attempt;saved.interval_base_time=base_time;saved.wall_interval_values=values;++frame;
            }
            previous=epoch;previous_time=time;last_attempt=attempt;++count;
            Require(count<=segment.row_count&&epoch<=segment.last_epoch,"Wall interval exceeds its segment");
        }
        Require(count==segment.row_count&&previous==segment.last_epoch,"Wall interval segment is incomplete");
    }
    Require(previous==b.info.final_epoch&&Bits(previous_time)==Bits(b.info.final_time)&&frame==b.entries.size(),"Wall intervals do not cover every accepted frame");
}
}
