#pragma once
#include "Run.h"
#include "case/vehicle_startup/joints/VehicleJointModel.h"
namespace crash::cases::vehicle_run {
struct PreparedRun::Data {
    Data(const vehicle_wall::VehicleWallSetup& s,const vehicle_runtime::JointModel& j,
        output::physical_frames::Mapping m,Config c,records::Identity id,ContactComposition composition)
        : setup(s),joints(j),mapping(std::move(m)),config(c),identity(std::move(id)),contact(std::move(composition)) {
        profile.beam18=s.execution().model().structural_beams()!=nullptr;
        profile.self_contact=c.contact_profile==ContactProfile::WallSelfContactV1;
    }
    vehicle_wall::VehicleWallSetup setup;
    vehicle_runtime::JointModel joints;
    output::physical_frames::Mapping mapping;
    Config config;
    records::Identity identity;
    ContactComposition contact;
    vehicle_dynamics::Config dynamics;
    Horizon horizon;
    Forecast forecast;
    records::source::BundleRequest request;
    output::physical_run::Profile profile{true,true};
};
namespace detail {
records::RecordFile WriteSummary(const std::filesystem::path&,const Config&,const Horizon&,
    const Forecast&,const Result&);
}
} // namespace crash::cases::vehicle_run
