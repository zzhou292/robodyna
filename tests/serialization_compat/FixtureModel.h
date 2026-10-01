// Robodyna serialization migration fixtures; existing Chrono APIs are deliberate.
#pragma once

#include <memory>

#include "chrono/core/ChFrame.h"
#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChBodyAuxRef.h"

namespace robodyna::serialization_compat {

// Include both value archives and a multiply inherited, shared polymorphic object.
// These are object archives, not a physical restart checkpoint.
struct FixtureModel {
    chrono::ChVector3d vector;
    chrono::ChFramed frame;
    chrono::ChBody plain_body;
    std::shared_ptr<chrono::ChBody> body;
    std::shared_ptr<chrono::ChBody> alias;
    std::shared_ptr<chrono::ChBodyFrame> frame_alias;
    std::shared_ptr<chrono::ChPhysicsItem> item_alias;
    std::shared_ptr<chrono::ChBody> null_body;

    void ArchiveOut(chrono::ChArchiveOut& archive);
    void ArchiveIn(chrono::ChArchiveIn& archive);
};

void PopulateFixture(FixtureModel& fixture);
void CheckFixture(FixtureModel& fixture);

}  // namespace robodyna::serialization_compat
