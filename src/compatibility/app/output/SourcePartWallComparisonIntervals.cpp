#include "SourcePartWallComparisonInput.h"
#include "SourcePartWallArtifactSchema.h"
#include "CsvLedgerSegments.h"
#include <charconv>
#include <sstream>

namespace crash::output::wall_comparison {
namespace {
template<class T> T Csv(const std::string& s) {
    T value{};const auto parsed=std::from_chars(s.data(),s.data()+s.size(),value);
    Require(parsed.ec==std::errc{}&&parsed.ptr==s.data()+s.size(),"Malformed wall comparison interval value");return value;
}
IntervalPoint Point(const std::array<std::string,SourcePartWallIntervalColumns>& row) {
    IntervalPoint p;
    p.epoch=Csv<std::uint64_t>(row[4]);p.time=Csv<double>(row[5]);
    p.reaction=Csv<double>(row[13]);p.reaction_error=Csv<double>(row[14]);
    p.potential=Csv<double>(row[15]);p.potential_error=Csv<double>(row[16]);
    p.impulse=Csv<double>(row[23]);p.impulse_error=Csv<double>(row[24]);
    p.penetration=Csv<double>(row[25]);p.active_nodes=Csv<unsigned>(row[26]);
    p.momentum_residual=Csv<double>(row[29]);p.momentum_allowance=Csv<double>(row[30]);
    p.physical_energy_uncertainty=Csv<double>(row[32]);p.strictly_separated_nodes=Csv<unsigned>(row[33]);return p;
}
void CheckEnergy(const Run& run,const std::array<std::string,SourcePartWallIntervalColumns>& row,const IntervalPoint& p) {
    const double kinetic=Csv<double>(row[8]),work=Csv<double>(row[9]),residual=Csv<double>(row[10]);
    const double allowance=Csv<double>(row[28]);
    const double budget=.05*run.scales.initial_kinetic+1e-10;
    Require(std::isfinite(kinetic)&&kinetic>=0&&std::isfinite(work)&&work>=-.01*run.scales.initial_kinetic&&
        std::isfinite(residual)&&Bits(allowance)==Bits(budget)&&std::abs(residual)+p.physical_energy_uncertainty<=budget,
        "Wall interval violates the frozen native-work or physical-energy gate");
    const long double independent=static_cast<long double>(kinetic)+work+p.potential-run.scales.initial_kinetic;
    const long double rounding=512*std::numeric_limits<double>::epsilon()*
        (std::abs(kinetic)+std::abs(work)+p.potential+run.scales.initial_kinetic);
    Require(std::abs(independent-residual)<=rounding,"Wall interval energy residual differs from its actual channels");
}
}
Events ReadEvents(const Run& run) {
    const auto& gap=Field(Field(run.configuration,"wall_setup"),"actual_leading_gap_interval_m");
    Require(gap.IsArray()&&gap.Size()==2&&gap[0].IsNumber()&&gap[1].IsNumber(),"Missing certified onset timing interval");
    const double gap_width=gap[1].GetDouble()-gap[0].GetDouble();
    EventTracker tracker(run.refinement,run.initial_momentum,gap_width/run.scales.speed);
    const auto plans=ParseCsvLedgerSegments(Field(run.manifest,kCsvLedgerSegmentsField));
    Require(plans.size()==1,"Wall comparison requires one accepted interval ledger");
    const auto& plan=plans[0];ValidateCsvLedgerPlan(plan,SourcePartWallIntervalHeader);
    Require(plan.interval_count==BaseHorizon*run.refinement,"Wall event ledger is an incomplete prefix");
    std::uint64_t previous_attempt=0;
    for(const auto& segment:plan.segments) {
        std::istringstream stream(ReadBounded(run.directory/segment.file,segment.byte_cap));std::string line;
        Require(bool(std::getline(stream,line))&&line+"\n"==SourcePartWallIntervalHeader,"Wall event ledger header changed");
        std::uint64_t count=0;
        while(std::getline(stream,line)) {
            std::array<std::string,SourcePartWallIntervalColumns> columns;std::istringstream row(line);
            for(auto& column:columns)Require(bool(std::getline(row,column,','))&&!column.empty(),"Truncated wall event interval");
            Require(row.eof(),"Unexpected wall event columns");const auto p=Point(columns);
            const auto attempt=Csv<std::uint64_t>(columns[2]);
            Require(Csv<std::uint64_t>(columns[0])==run.replay.info()->owner_id&&
                Csv<std::uint64_t>(columns[1])+1==p.epoch&&attempt>previous_attempt&&
                p.epoch>=segment.first_epoch&&p.epoch<=segment.last_epoch,"Wall event interval association changed");
            CheckEnergy(run,columns,p);tracker.Observe(p);previous_attempt=attempt;++count;
        }
        Require(count==segment.row_count&&tracker.events().final_epoch==segment.last_epoch,"Incomplete wall event segment");
    }
    Require(tracker.events().final_epoch==plan.interval_count,"Incomplete wall event ledger");return tracker.events();
}
void CheckFrameMomentum(const Run& run,const Value& f) {
    const auto& velocity=Field(f,"velocity_xyz_m_per_s");
    Require(velocity.IsArray()&&velocity.Size()==351,"Missing raw carried source velocity");
    long double carried=0,absolute=0;
    for(unsigned n=0;n<117;++n) {
        Require(velocity[3*n].IsNumber(),"Invalid raw carried normal velocity");
        const long double term=static_cast<long double>(run.mass[n])*velocity[3*n].GetDouble();
        Require(std::isfinite(term),"Nonfinite native mass-weighted momentum");carried+=term;absolute+=std::abs(term);
    }
    const double impulse=Real(f,"cumulative_wall_kick_impulse_N_s");
    const double error=Real(f,"cumulative_wall_kick_impulse_error_N_s");
    const double residual=Field(f,"carried_momentum_residual_xyz_kg_m_per_s")[0].GetDouble();
    const double allowance=Field(f,"carried_momentum_allowance_xyz_kg_m_per_s")[0].GetDouble();
    const auto rounding=MomentumArithmetic(run.initial_momentum,impulse,residual)+
        64*std::numeric_limits<double>::epsilon()*absolute;
    Require(std::abs(carried-(run.initial_momentum-impulse+residual))<=rounding+error+allowance,
        "Saved raw native COM momentum disagrees with the interval impulse reconstruction");
    if(run.events.rebound.observed&&Unsigned(f,"accepted_epoch")>=run.events.terminal_separation_start)
        Require(carried+rounding<0,"Saved raw COM does not support the certified terminal rebound");
}
} // namespace crash::output::wall_comparison
