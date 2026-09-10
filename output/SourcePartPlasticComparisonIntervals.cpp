#include "SourcePartPlasticComparisonInput.h"
#include "SourcePartWallArtifactSchema.h"
#include "CsvLedgerSegments.h"
#include <charconv>
#include <sstream>

namespace crash::output::plastic_comparison {
namespace {
template<class T> T Csv(const std::string& text) {
    T value{};const auto result=std::from_chars(text.data(),text.data()+text.size(),value);
    Require(result.ec==std::errc{}&&result.ptr==text.data()+text.size(),"Invalid plastic interval scalar");return value;
}
using Row=std::array<std::string,SourcePartWallIntervalColumns>;
wc::IntervalPoint Point(const Row& row) {
    wc::IntervalPoint p;
    p.epoch=Csv<std::uint64_t>(row[4]);p.time=Csv<double>(row[5]);
    p.reaction=Csv<double>(row[13]);p.reaction_error=Csv<double>(row[14]);
    p.potential=Csv<double>(row[15]);p.potential_error=Csv<double>(row[16]);
    p.impulse=Csv<double>(row[23]);p.impulse_error=Csv<double>(row[24]);
    p.penetration=Csv<double>(row[25]);p.active_nodes=Csv<unsigned>(row[26]);
    p.momentum_residual=Csv<double>(row[29]);p.momentum_allowance=Csv<double>(row[30]);
    p.physical_energy_uncertainty=Csv<double>(row[32]);p.strictly_separated_nodes=Csv<unsigned>(row[33]);return p;
}
void Energy(const Run& run,const Row& row,const wc::IntervalPoint& p,Observations& observation) {
    const double kinetic=Csv<double>(row[8]),work=Csv<double>(row[9]),residual=Csv<double>(row[10]);
    const double budget=wc::Real(run.configuration,"maximum_energy_residual_J")+
        wc::Real(run.configuration,"relative_energy_residual")*run.initial_kinetic;
    const double upper=std::abs(residual)+p.physical_energy_uncertainty;
    Require(std::isfinite(kinetic)&&kinetic>=0&&std::isfinite(work)&&work>=-.01*run.initial_kinetic&&
        std::isfinite(residual)&&std::isfinite(upper)&&budget>0&&
        Bits(Csv<double>(row[28]))==Bits(budget)&&upper<=budget,
        "Plastic interval violates its declared native-work or physical-energy bound");
    const long double independent=static_cast<long double>(kinetic)+work+p.potential-run.initial_kinetic;
    const long double rounding=512*std::numeric_limits<double>::epsilon()*
        (std::abs(kinetic)+std::abs(work)+p.potential+run.initial_kinetic);
    Require(std::abs(independent-residual)<=rounding,"Plastic interval energy channels disagree");
    observation.maximum_absolute_energy_residual=std::max(observation.maximum_absolute_energy_residual,std::abs(residual));
    observation.maximum_energy_bound_fraction=std::max(observation.maximum_energy_bound_fraction,upper/budget);
    observation.final_raw_com_velocity=static_cast<double>(
        (run.initial_momentum-p.impulse+p.momentum_residual)/run.total_mass);
}
}
Observations ReadIntervals(const Run& run) {
    Observations observed;
    const auto& gap=wc::Field(wc::Field(run.configuration,"wall_setup"),"actual_leading_gap_interval_m");
    Require(gap.IsArray()&&gap.Size()==2,"Missing source wall gap certificate");
    const double uncertainty=(Scalar(gap[1])-Scalar(gap[0]))/run.speed;
    wc::EventTracker events(run.refinement,run.initial_momentum,uncertainty);
    const auto plans=ParseCsvLedgerSegments(wc::Field(run.manifest,kCsvLedgerSegmentsField));
    Require(plans.size()==1,"Plastic report requires one actual accepted interval ledger");
    const auto& plan=plans[0];ValidateCsvLedgerPlan(plan,SourcePartWallIntervalHeader);
    Require(plan.interval_count==run.replay.info()->final_epoch,"Plastic interval prefix differs from accepted endpoint");
    std::uint64_t previous_attempt=0;
    for(const auto& segment:plan.segments) {
        std::istringstream input(ReadBounded(run.directory/segment.file,segment.byte_cap));std::string line;
        Require(bool(std::getline(input,line))&&line+"\n"==SourcePartWallIntervalHeader,"Plastic interval header changed");
        std::uint64_t count=0;
        while(std::getline(input,line)) {
            Row row;std::istringstream columns(line);
            for(auto& value:row)Require(bool(std::getline(columns,value,','))&&!value.empty(),"Incomplete plastic interval row");
            Require(columns.eof(),"Unexpected plastic interval columns");
            const auto p=Point(row);const auto attempt=Csv<std::uint64_t>(row[2]);
            Require(Csv<std::uint64_t>(row[0])==run.replay.info()->owner_id&&Csv<std::uint64_t>(row[1])+1==p.epoch&&
                attempt>previous_attempt&&p.epoch>=segment.first_epoch&&p.epoch<=segment.last_epoch,
                "Plastic interval owner/epoch/attempt association changed");
            Energy(run,row,p,observed);events.Observe(p);previous_attempt=attempt;++count;
        }
        Require(count==segment.row_count&&events.events().final_epoch==segment.last_epoch,"Incomplete plastic interval segment");
    }
    Require(events.events().final_epoch==plan.interval_count,"Incomplete plastic interval ledger");
    observed.events=events.events();return observed;
}
} // namespace crash::output::plastic_comparison
