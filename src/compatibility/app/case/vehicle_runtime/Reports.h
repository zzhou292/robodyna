#pragma once
#include "output/ArtifactIO.h"
namespace crash::cases::vehicle_runtime::detail {
template<class Report> void RequireSuccess(const Report& r) {
    // All native report enums used here define their successful status first.
    output::Require(static_cast<int>(r.status)==0,r.message);
}
} // namespace crash::cases::vehicle_runtime::detail
