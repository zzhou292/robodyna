#pragma once
#include "NativeSceneRun.h"
#include "output/physical_run/ViewerInput.h"
namespace crash::cases::native_scene {
struct PreparedNativeSceneRun::Data {
    Data(const ContactSource& c,const ArchiveSource& a,RunConfig config,output::full_shell::Context context)
      :contact(c),archive_source(a),config(config),prospective(std::move(context)){}
    ContactSource contact;ArchiveSource archive_source;RunConfig config;
    output::full_shell::Context prospective;
    vehicle_run::Horizon horizon;RunForecast forecast;
    output::full_shell::source::BundleRequest request;
};
namespace run_detail {
output::Document ForecastDocument(const RunConfig&,const vehicle_run::Horizon&,const RunForecast&,const ContactSource&);
output::full_shell::RecordFile Summary(const std::filesystem::path&,const RunConfig&,const vehicle_run::Horizon&,
    const RunForecast&,const ContactSource&,const RunResult&);
}
}
