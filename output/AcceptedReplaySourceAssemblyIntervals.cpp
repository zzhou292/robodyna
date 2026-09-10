#include "AcceptedReplaySourceAssembly.h"
#include "AcceptedReplayCsv.h"
#include "source_assembly/SourceAssemblyWallSchema.h"
#include <set>

namespace crash::output::replay_detail {
void ReadAssemblyIntervals(Bundle& b,const Document& config,const Document& manifest) {
    using namespace output::assembly;
    auto& a=*b.assembly;
    const auto planned=ParseCsvLedgerSegments(Member(config,kCsvLedgerSegmentsField));
    const auto completed=ParseCsvLedgerSegments(Member(manifest,kCsvLedgerSegmentsField));
    Require(planned.size()==1&&completed.size()==1,"Assembly replay requires one interval ledger");
    const auto file_cap=Unsigned(config,"artifact_file_byte_cap");
    Require(file_cap&&file_cap<=kFileCap,"Invalid explicit assembly ledger file cap");
    ValidateCsvLedgerPlan(planned[0],WallIntervalHeader,file_cap);ValidateCsvLedgerPlan(completed[0],WallIntervalHeader,file_cap);
    Require(SameCsvLedgerPlan(planned[0],PlanCsvLedger("accepted-intervals.csv",WallIntervalHeader,
        a.requested_steps,WallIntervalColumns*26,file_cap)),"Assembly interval reservation differs from the shared writer plan");
    const auto& plan=completed[0];
    Require(planned[0].logical_file=="accepted-intervals.csv"&&planned[0].interval_count==a.requested_steps&&
        SameCsvLedgerPlan(plan,PlanCsvLedger(planned[0].logical_file,WallIntervalHeader,b.info.final_epoch,planned[0].max_row_bytes,file_cap)),
        "Assembly ledger is not the declared accepted prefix");
    std::uint64_t previous=0,last_attempt=0;double previous_time=0,previous_velocity=0,previous_base=0;
    double previous_plastic=0,previous_work=0,previous_potential=0;
    std::array<std::uint64_t,3> history{};std::size_t frame=1;a.intervals.resize(b.entries.size());
    a.interval_files.clear();
    for(const auto& segment:plan.segments) {
        a.interval_files.push_back(segment.file);
        std::istringstream input(VerifiedBytes(b,segment.file));std::string line;
        Require(bool(std::getline(input,line))&&line+"\n"==WallIntervalHeader,"Assembly interval columns changed");
        std::uint64_t count=0;
        while(std::getline(input,line)) {
            Require(line.size()+1<=plan.max_row_bytes,"Assembly interval exceeds declared row cap");
            const auto row=ReplayCsvRow<WallIntervalColumns>(line);
            const auto owner=ReplayCsvNumber<std::uint64_t>(row[0]),base=ReplayCsvNumber<std::uint64_t>(row[1]);
            const auto attempt=ReplayCsvNumber<std::uint64_t>(row[2]),epoch=ReplayCsvNumber<std::uint64_t>(row[4]);
            const double base_time=ReplayCsvNumber<double>(row[3]),time=ReplayCsvNumber<double>(row[5]);
            std::array<double,33> v{};
            for(unsigned j=0;j<v.size();++j) {v[j]=ReplayCsvNumber<double>(row[j+6]);Require(std::isfinite(v[j]),"Nonfinite assembly interval value");}
            Require(owner==b.info.owner_id&&base==previous&&epoch==base+1&&attempt>last_attempt&&
                Bits(base_time)==Bits(previous_time)&&Bits(time)==Bits(base_time+b.fixed_dt)&&
                Bits(v[0])==Bits(base_time+.5*b.fixed_dt)&&Bits(v[1])==Bits(base==0?.5*b.fixed_dt:b.fixed_dt),
                "Assembly interval owner, accepted phase or attempt changed");
            for(unsigned j:{2u,3u,5u,13u,14u,15u,16u,17u,20u,21u,23u,24u,25u,26u,27u,28u,29u,30u,31u,32u})
                Require(v[j]>=0,"Negative assembly interval magnitude");
            Require(v[25]<=b.wall_penetration_cap&&v[26]<=b.info.node_count&&std::floor(v[26])==v[26]&&
                v[28]<=3*a.source.data().parents.size()&&std::floor(v[28])==v[28]&&
                v[29]<=a.source.data().parents.size()&&std::floor(v[29])==v[29]&&
                v[30]<=AssemblyQuaternionLimit(a)&&v[31]<=a.maximum_area_ratio&&v[32]<=a.maximum_thickness_ratio&&
                v[27]>=previous_plastic&&v[5]>=previous_work,"Assembly interval count, envelope or plastic history changed");
            Require(std::abs(v[11])<=v[13]&&std::abs(v[12])<=v[13],"Assembly native recurrence/bookkeeping budget exceeded");
            if(v[26]>0) {if(!history[0])history[0]=epoch;history[1]=epoch;++history[2];}
            if(frame<b.entries.size()&&b.entries[frame].epoch==epoch) {
                auto& e=b.entries[frame];AssemblyEqual(e.time,time);e.interval_attempt=attempt;e.interval_base_time=base_time;
                e.interval_base_velocity_time=previous_velocity;e.interval_previous_base_time=previous_base;
                e.interval_base_wall_potential=previous_potential;e.interval_contact_history=history;
                a.intervals[frame]=v;++frame;
            }
            previous=epoch;previous_time=time;previous_velocity=v[0];previous_base=base_time;
            previous_plastic=v[27];previous_work=v[5];previous_potential=v[16];last_attempt=attempt;++count;
            Require(count<=segment.row_count&&epoch<=segment.last_epoch,"Assembly interval exceeds its segment");
        }
        Require(count==segment.row_count&&previous==segment.last_epoch,"Assembly interval segment is incomplete");
    }
    Require(previous==b.info.final_epoch&&Bits(previous_time)==Bits(b.info.final_time)&&frame==b.entries.size(),
        "Assembly intervals do not cover every accepted frame");
}
} // namespace crash::output::replay_detail
