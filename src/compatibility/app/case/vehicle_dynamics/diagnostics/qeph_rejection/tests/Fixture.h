#pragma once
#include "../Capture.h"
#include <cstring>
#include "lib_src/elements/qeph/QephStartup.h"
#include "lib_src/elements/qeph/QephHistory.h"
#include "output/full_shell/tests/TestSupport.h"
namespace crash::cases::vehicle_dynamics::diagnostics::qeph_rejection::test {
inline native::RejectedCandidateInput Input() {
    native::RejectedCandidateInput input;
    native::ReferenceInput reference;
    reference.position[0] = {0, 0, 0}; reference.position[1] = {1, 0, 0};
    reference.position[2] = {1, 1, 0}; reference.position[3] = {0, 1, 0};
    EXPECT_EQ(native::InitializeReference(reference, input.element.reference), native::Status::kSuccess);
    for (unsigned i = 0; i < 4; ++i) {
        input.element.nodes[i] = i;
        input.interval.position_endpoint[i] = reference.position[i];
    }
    EXPECT_EQ(native::InitializeHistory(input.element.reference, {.25, 2},
        input.accepted_force.proposed_history), native::Status::kSuccess);
    input.metadata.original = {native::BatchStatus::ElementFailure, 12, UINT32_MAX,
        native::Status::kInvalidInput, tl::fea::NodalStatus::Ok};
    input.metadata.owner.owner_id = 77;
    input.metadata.owner.epoch = 2;
    input.metadata.owner.node_count = 4;
    input.metadata.owner.time = .25;
    input.metadata.owner.fixed_dt = .125;
    input.metadata.accepted.owner_id = 77;
    input.metadata.accepted.epoch = 2;
    input.metadata.candidate.owner_id = 77;
    input.metadata.candidate.base_epoch = 2;
    input.metadata.candidate.epoch = 3;
    input.metadata.candidate.attempt = 4;
    input.metadata.candidate.valid = false;
    input.source = {true, 101, 102, 103, 104};
    input.interval.base_time = .25;
    input.interval.dt = .125;
    input.interval.sample_index = 3;
    input.route = native::RejectedCandidateRoute::PlainForce;
    return input;
}
inline native::BatchReport Original() {
    return {native::BatchStatus::ElementFailure, "QEPH device validation failed", 12,
        UINT32_MAX, native::Status::kInvalidInput, tl::fea::NodalStatus::Ok};
}
inline CaptureState Captured() {
    CaptureState state;
    state.input = Input();
    const std::uint64_t nan = UINT64_C(0x7ff800000000cafe);
    std::memcpy(&state.input.interval.position_endpoint[0].x, &nan, sizeof(nan));
    native::RejectedCaptureReport report;
    report.status = native::RejectedCaptureStatus::Captured;
    report.metadata_available = true;
    report.metadata = state.input.metadata;
    SourceRequest request{state.input.source, true, state.input.law};
    FinishCapture(state, Original(), report, request);
    EXPECT_TRUE(state.complete);
    return state;
}
}
