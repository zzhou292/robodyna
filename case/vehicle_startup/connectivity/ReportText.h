#pragma once
#include "Components.h"
#include <string>
namespace crash::cases::vehicle_startup::connectivity::detail {
struct ReportIdentity {
    std::string archive_sha256, member_sha256, canonical_sha256, tire_policy;
};
std::string RenderReport(const Data&, tl::util::ConstView<tl::fea::NodalDomainNode>,
                         const Forecast&, const ReportIdentity&, std::size_t cap);
} // namespace crash::cases::vehicle_startup::connectivity::detail
