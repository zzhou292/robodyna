#pragma once
#include "../MechanicsSummary.h"
#include "../MechanicsDocument.h"
#include "chrono_thirdparty/rapidjson/stringbuffer.h"
#include "chrono_thirdparty/rapidjson/writer.h"

namespace crash::cases::vehicle_run::test {
inline tl::fea::NodalStamp MechanicsStamp(std::uint64_t epoch) {
    tl::fea::NodalStamp stamp;
    stamp.owner_id = 7;
    stamp.epoch = epoch;
    stamp.node_count = 100;
    stamp.fixed_dt = .125;
    stamp.time = epoch * .125;
    stamp.velocity_time = epoch ? stamp.time - .0625 : 0;
    stamp.reactions_valid = epoch != 0;
    stamp.reaction_base_epoch = epoch ? epoch - 1 : 0;
    stamp.reaction_time = epoch ? stamp.time - .125 : 0;
    stamp.reaction_kick_dt = epoch == 1 ? .0625 : .125;
    return stamp;
}
template<class Diagnostics>
void MechanicsIdentity(Diagnostics& row, std::uint64_t epoch) {
    const auto stamp = MechanicsStamp(epoch);
    row.valid = true;
    row.has_completed_interval = true;
    row.phase = decltype(row.phase)::Prepared;
    row.owner_id = stamp.owner_id;
    row.configuration_id = 11;
    row.qualification_id = 13;
    row.epoch = epoch;
    row.base_epoch = epoch - 1;
    row.attempt = epoch + 3; // Earlier discarded attempts do not get counted.
    row.time = stamp.time;
    row.base_time = stamp.reaction_time;
    row.velocity_time = stamp.velocity_time;
    row.kick_dt = stamp.reaction_kick_dt;
}
inline vehicle_dynamics::StepObservation MechanicsStep(std::uint64_t epoch, bool beams = true) {
    vehicle_dynamics::StepObservation step;
    step.base = MechanicsStamp(epoch - 1);
    step.proposed_time = MechanicsStamp(epoch).time;
    auto& mechanics = step.mechanics;
    mechanics.base_stamp = step.base;
    mechanics.valid = true;
    mechanics.has_qeph = mechanics.has_t3 = mechanics.has_qbat = true;
    mechanics.has_type25 = mechanics.has_type13 = mechanics.has_solids = true;
    mechanics.has_type45 = true;
    mechanics.has_beam18 = beams;
    MechanicsIdentity(mechanics.qeph, epoch);
    MechanicsIdentity(mechanics.t3, epoch);
    MechanicsIdentity(mechanics.qbat, epoch);
    MechanicsIdentity(mechanics.type25, epoch);
    MechanicsIdentity(mechanics.type13, epoch);
    MechanicsIdentity(mechanics.solids, epoch);
    MechanicsIdentity(mechanics.type45, epoch);
    MechanicsIdentity(mechanics.beam18, epoch);
    auto& solids = mechanics.solids;
    solids.source_instance_id = 17;
    solids.accepted_force_assembled = true;
    solids.base_velocity_time = step.base.velocity_time;
    for (unsigned family = 0; family < 5; ++family) {
        solids.parent_count[family] = family + 1;
        solids.native_internal_work_increment_j[family] = 1 + family;
    }
    solids.physical_hourglass_work_increment_j[1] = .25;
    solids.physical_hourglass_work_increment_j[2] = -.125;
    solids.internal_kick_work_j = -.5;
    solids.internal_drift_work_j = -.75;
    solids.minimum_native_dt_s = .5;
    auto& beam = mechanics.beam18;
    beam.source_instance_id = 17;
    beam.parent_count = 4;
    beam.accepted_force_assembled = true;
    beam.base_velocity_time = step.base.velocity_time;
    beam.native_internal_work_increment_j[0] = 4;
    beam.native_internal_work_increment_j[1] = -2;
    beam.internal_kick_work_j = -.25;
    beam.internal_drift_work_j = -.375;
    beam.minimum_native_dt_s = .75;
    step.uniform_motion = {100, .01, .02, .03, .04};
    return step;
}
inline std::string MechanicsJson(const MechanicsTotals& totals) {
    auto document = detail::MechanicsDocument(totals);
    rapidjson::StringBuffer buffer;
    rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
    document.Accept(writer);
    return {buffer.GetString(), buffer.GetSize()};
}
} // namespace crash::cases::vehicle_run::test
