#pragma once
#include "VehiclePhysicalStartup.h"
#include "ParticipantConfigs.h"
#include "SourceRoles.h"
namespace crash::cases::vehicle_runtime {
struct VehiclePhysicalStartup::Storage {
    Storage(const Execution& e,const Attachments& a,Config c,Forecast f)
        : execution(e),attachments(a),config(c),forecast(f) {}
    // Reverse destruction releases the publication claim, participants, then
    // the sole owner. All immutable source handles outlive those consumers.
    Execution execution;
    Attachments attachments;
    Config config;
    Forecast forecast;
    SourceRoles roles;
    tl::fea::FENodalState owner;
    tl::fea::qeph::QephBatch qeph;
    tl::fea::t3::T3Batch t3;
    tl::fea::qbat::Batch qbat;
    tl::fea::type25::Batch type25;
    tl::fea::type13::Batch type13;
    tl::fea::solids::Batch solids;
    tl::fea::ShellBatchPublication publication;
    void InitializeOwner();
    void InitializeParticipants();
    void BindInitialCaches();
    void InitializePublication();
    tl::fea::NodalAllocationInfo Allocations() const noexcept;
    void InspectOwner(InitialInspection&);
    void InspectShells(InitialInspection&);
    void InspectConnections(InitialInspection&);
    void InspectSolids(InitialInspection&);
};
} // namespace crash::cases::vehicle_runtime
