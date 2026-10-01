#pragma once
#include <memory>
#include "chrono/physics/ChBody.h"

namespace robodyna::body_compat {
struct BodyGraph {
    std::shared_ptr<chrono::ChBody> body;
    std::shared_ptr<chrono::ChMarker> marker_alias;
    std::shared_ptr<chrono::ChForce> force_alias;
    void ArchiveOut(chrono::ChArchiveOut& archive);
    void ArchiveIn(chrono::ChArchiveIn& archive);
};

struct Fixture {
    BodyGraph plain;
    BodyGraph easy;
    void ArchiveOut(chrono::ChArchiveOut& archive);
    void ArchiveIn(chrono::ChArchiveIn& archive);
};

void Populate(Fixture& fixture);
void Check(Fixture& fixture);
}  // namespace robodyna::body_compat
