#include "ActivityValues.h"
#include "output/ArtifactIO.h"
#include "lib_src/solvers/NodalTrialIdentity.h"
namespace crash::cases::vehicle_startup::cin_witness_detail {
namespace {
using Status=TiedCinActivityStatus;
const std::uint8_t* Value(const TiedCinWitnessOrigin& origin,const cin_stage::ActiveWitness& witness,
                         FamilyActivity fields) {
    using Family=tl::fea::ShellBindingFamily;
    if (origin.family==Family::Qeph && witness.native_parent_index<fields.qeph_count && fields.qeph)
        return fields.qeph+witness.native_parent_index;
    if (origin.family==Family::T3 && witness.native_parent_index<fields.t3_count && fields.t3)
        return fields.t3+witness.native_parent_index;
    if (origin.family==Family::Qbat && witness.native_parent_index<fields.qbat_count && fields.qbat)
        return fields.qbat+witness.native_parent_index;
    return nullptr;
}
}
TiedCinActivityReport MapActivity(const TiedCinWitnessData& roster,FamilyActivity fields,
        std::uint8_t* output,std::size_t capacity) {
    if ((!output && capacity) || capacity!=roster.witnesses.size() || roster.origins.size()!=capacity ||
        roster.counts.witnesses!=capacity || roster.ranges.size()!=roster.counts.attachments)
        return {Status::InvalidInput,"CIN activity map count differs from the complete roster"};
    if (fields.qeph_count>524288 || fields.t3_count>524288 || fields.qbat_count>524288 ||
        fields.qeph_count+fields.t3_count+fields.qbat_count!=roster.counts.source_parents)
        return {Status::InvalidInput,"Family activity counts differ from the complete shell source"};
    if (capacity) {
        using tl::fea::trial_identity::Disjoint;
        const auto disjoint=[&](const void* pointer,std::size_t bytes) {
            return !bytes || Disjoint(output,capacity,pointer,bytes);
        };
        if (!disjoint(fields.qeph,fields.qeph_count) || !disjoint(fields.t3,fields.t3_count) ||
            !disjoint(fields.qbat,fields.qbat_count) ||
            !disjoint(roster.witnesses.data(),roster.witnesses.size()*sizeof(cin_stage::ActiveWitness)) ||
            !disjoint(roster.origins.data(),roster.origins.size()*sizeof(TiedCinWitnessOrigin)) ||
            !disjoint(roster.ranges.data(),roster.ranges.size()*sizeof(cin_stage::WitnessRange)))
            return {Status::InvalidInput,"Activity map output overlaps inspected inputs"};
    }
    std::size_t end=0;
    for (const auto& range:roster.ranges) {
        if (range.offset!=end || range.count>capacity-end)
            return {Status::InvalidInput,"CIN activity map has an invalid ordered range"};
        end+=range.count;
    }
    if (end!=capacity) return {Status::InvalidInput,"CIN activity map has trailing values"};
    for (std::size_t i=0;i<capacity;++i) {
        const auto* flag=Value(roster.origins[i],roster.witnesses[i],fields);
        if (!flag || *flag>1) return {Status::InvalidInput,"Invalid actual parent activity association",i};
    }
    TiedCinActivityReport result{Status::Success,"All attachments have a positive accepted shell witness"};
    for (std::size_t row=0;row<roster.ranges.size();++row) {
        const auto& range=roster.ranges[row];
        bool active=false;
        for (std::size_t i=range.offset;i<range.offset+range.count;++i) {
            const bool parent_active=*Value(roster.origins[i],roster.witnesses[i],fields)!=0;
            output[i]=parent_active ? 1 : 2;
            active=active || parent_active;
        }
        if (!active && result.status==Status::Success)
            result={Status::PendingPositiveShellWitness,"No positive accepted shell witness; release remains unqualified",row};
    }
    return result;
}
TiedCinActivityForecast ActivityBudget(std::size_t retained,std::size_t parents,
        std::size_t witnesses,std::size_t fixed,TiedCinActivityLimits limits) {
    output::Require(limits.workspace_bytes && limits.workspace_bytes<=TiedCinActivityLimits{}.workspace_bytes &&
        parents && parents<=524288 && witnesses<=262144,"Invalid CIN activity workspace limits");
    TiedCinActivityForecast next;
    next.retained_roster_bound=retained;
    const auto add=[&](std::size_t bytes) {
        output::Require(bytes<=limits.workspace_bytes-next.workspace_bytes,"CIN activity workspace byte cap exceeded");
        next.workspace_bytes+=bytes;
    };
    add(fixed);
    add(parents);
    add(witnesses);
    add(witnesses);
    output::Require(retained<=TiedCinWitnessLimits{}.host_bytes-next.workspace_bytes,
                    "CIN complete activity reservation exceeds existing source host cap");
    next.total_host_bytes=retained+next.workspace_bytes;
    return next;
}
}
