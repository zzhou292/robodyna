#include "tests/system_compat/SystemFixture.h"
#include "chrono/physics/ChContactContainerNSC.h"
#include "chrono/physics/ChContactContainerSMC.h"

#include <stdexcept>

namespace robodyna::system_compat {
using chrono::make_ChNameValue;

void Fixture::ArchiveOut(chrono::ChArchiveOut& archive) {
    archive << CHNVP(nsc) << CHNVP(smc) << CHNVP(nsc_body) << CHNVP(smc_body);
}
void Fixture::ArchiveIn(chrono::ChArchiveIn& archive) {
    archive >> CHNVP(nsc) >> CHNVP(smc) >> CHNVP(nsc_body) >> CHNVP(smc_body);
}

namespace {
void Require(bool value, const char* message) {
    if (!value)
        throw std::runtime_error(message);
}

std::shared_ptr<chrono::ChBody> Configure(chrono::ChSystem& system, double time, int tag) {
    system.SetNumThreads(1, 1, 1);
    system.SetGravitationalAcceleration({.5, -1.25, .125});
    system.SetChTime(time);
    system.SetSleepingAllowed(true);
    system.SetMaxPenetrationRecoverySpeed(.625);
    auto body = chrono_types::make_shared<chrono::ChBody>();
    body->SetName("archived system body");
    body->SetTag(tag);
    body->SetMass(2.5);
    body->SetInertiaXX({1, 2, 3});
    body->SetPos({1, -2, .5});
    body->SetPosDt({.125, -.25, .5});
    body->SetChTime(time);
    system.AddBody(body);
    system.Setup();
    return body;
}

void CheckSystem(chrono::ChSystem& system, const std::shared_ptr<chrono::ChBody>& body,
                 double time, int tag, const char* archive_tag, bool restored) {
    Require(chrono::ChClassFactory::GetClassTagName(typeid(system)) == archive_tag,
            "system factory tag changed");
    Require(system.GetChTime() == time && system.GetStep() == .04 && system.GetNumSteps() == 0,
            "represented system time/step metadata changed");
    Require((system.GetGravitationalAcceleration() - chrono::ChVector3d(.5, -1.25, .125)).Length2() == 0 &&
                system.IsSleepingAllowed(), "represented gravity/sleep settings changed");
    Require(system.GetSolverType() == chrono::ChSolver::Type::PSOR,
            "represented solver type changed");
    Require(system.GetBodies().size() == 1 && body && system.GetBodies()[0] == body,
            "assembly body identity changed");
    Require(body->GetSystem() == &system && body->GetMass() == 2.5 && body->GetTag() == tag,
            "assembly owner or body values changed");
    Require(body->GetChTime() == time && body->GetName() == "archived system body" &&
                (body->GetPos() - chrono::ChVector3d(1, -2, .5)).Length2() == 0 &&
                (body->GetPosDt() - chrono::ChVector3d(.125, -.25, .5)).Length2() == 0,
            "archived body state changed");
    Require(system.GetContactContainer()->GetSystem() == &system,
            "constructor contact container lost its system owner");
    // Current system archives omit m_name; do not accidentally claim its recovery.
    Require(system.GetName() == (restored ? "" : archive_tag), "system-name archive scope changed");
}
}  // namespace

void Populate(Fixture& fixture) {
    fixture.nsc = chrono_types::make_shared<chrono::ChSystemNSC>("ChSystemNSC");
    fixture.smc = chrono_types::make_shared<chrono::ChSystemSMC>("ChSystemSMC");
    fixture.nsc_body = Configure(*fixture.nsc, .125, 31);
    fixture.smc_body = Configure(*fixture.smc, .25, 47);
    fixture.nsc->SetMinBounceSpeed(.375);
    fixture.smc->UseMaterialProperties(false);
    fixture.smc->SetContactForceModel(chrono::ChSystemSMC::Hooke);
    fixture.smc->SetAdhesionForceModel(chrono::ChSystemSMC::AdhesionForceModel::DMT);
    fixture.smc->SetTangentialDisplacementModel(chrono::ChSystemSMC::MultiStep);
    fixture.smc->SetSlipVelocityThreshold(.02);
    fixture.smc->SetCharacteristicImpactVelocity(4);
    fixture.smc->SetContactStiff(true);
}

void Check(Fixture& fixture, bool restored) {
    Require(fixture.nsc && fixture.smc, "system archive lost a concrete system");
    CheckSystem(*fixture.nsc, fixture.nsc_body, .125, 31, "ChSystemNSC", restored);
    CheckSystem(*fixture.smc, fixture.smc_body, .25, 47, "ChSystemSMC", restored);
    auto nsc_contacts = std::dynamic_pointer_cast<chrono::ChContactContainerNSC>(fixture.nsc->GetContactContainer());
    Require(nsc_contacts && std::dynamic_pointer_cast<chrono::ChContactContainerSMC>(fixture.smc->GetContactContainer()),
            "system contact-method constructor identity changed");
    // Contact container data and the stiff-contact switch are currently omitted.
    if (!restored)
        Require(nsc_contacts->GetMinBounceSpeed() == .375, "explicit NSC bounce setting changed");
    // The inherited NSC constructor does not initialize min_bounce_speed, and
    // the system archive omits that container. Do not read the restored field.
    Require(fixture.smc->IsContactStiff() == !restored, "SMC stiff-contact archive scope changed");
    Require(!fixture.smc->UsingMaterialProperties() &&
                fixture.smc->GetContactForceModel() == chrono::ChSystemSMC::Hooke &&
                fixture.smc->GetAdhesionForceModel() == chrono::ChSystemSMC::AdhesionForceModel::DMT &&
                fixture.smc->GetTangentialDisplacementModel() == chrono::ChSystemSMC::MultiStep &&
                fixture.smc->GetSlipVelocityThreshold() == .02 &&
                fixture.smc->GetCharacteristicImpactVelocity() == 4,
            "represented SMC settings changed");
}
}  // namespace robodyna::system_compat
