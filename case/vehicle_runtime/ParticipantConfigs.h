#pragma once
#include "Config.h"
#include "SourceIdentity.h"
namespace crash::cases::vehicle_runtime::detail {
struct ParticipantConfigs {
    tl::fea::qeph::QephBatchConfig qeph;
    tl::fea::t3::T3BatchConfig t3;
    tl::fea::qbat::BatchConfig qbat;
    tl::fea::type25::BatchConfig type25;
    tl::fea::type13::BatchConfig type13;
    tl::fea::solids::BatchConfig solids;
    tl::fea::ShellPhysicalPublicationIdentity publication;
};
ParticipantConfigs ConfigureParticipants(const Config&,const Execution&,const Attachments&,
                                        const tl::fea::NodalStamp&);
tl::fea::NodalStamp DescriptiveStamp(const Config&,const Execution&) noexcept;
tl::fea::NodalCinStartup CinStartup(const Config&,const Attachments&,
                                   const double* mass=nullptr,const double* inertia=nullptr) noexcept;
} // namespace crash::cases::vehicle_runtime::detail
