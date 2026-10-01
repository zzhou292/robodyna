#pragma once

#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChSystemSMC.h"

#include <memory>

namespace robodyna::system_compat {

// Concrete existing system types exercise their own registered archive tags.
// Body aliases also check that assembly loading preserves shared identity.
struct Fixture {
    std::shared_ptr<chrono::ChSystemNSC> nsc;
    std::shared_ptr<chrono::ChSystemSMC> smc;
    std::shared_ptr<chrono::ChBody> nsc_body;
    std::shared_ptr<chrono::ChBody> smc_body;
    void ArchiveOut(chrono::ChArchiveOut& archive);
    void ArchiveIn(chrono::ChArchiveIn& archive);
};

void Populate(Fixture& fixture);
void Check(Fixture& fixture, bool restored);

}  // namespace robodyna::system_compat
