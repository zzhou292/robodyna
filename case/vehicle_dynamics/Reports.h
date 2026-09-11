#pragma once
#include "output/ArtifactIO.h"
#include <stdexcept>
#include <string>
namespace crash::cases::vehicle_dynamics::detail {
template<class Report> void Require(const Report& r,const char* stage) {
    if(static_cast<int>(r.status)!=0) throw std::runtime_error(std::string(stage)+": "+r.message);
}
} // namespace crash::cases::vehicle_dynamics::detail
