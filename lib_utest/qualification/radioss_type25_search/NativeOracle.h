#pragma once
#include "lib_src/collision/radioss_type25/search/Values.h"
#include <vector>
namespace type25_search_test {
namespace s=tlfea::contact::radioss_type25::search;
s::Budget NativeBudget(const s::Extrema&,double margin,double dt,bool force);
s::Extrema NativeExtrema(const s::Source&,const s::Current& host_current,
    const s::Current& host_reference, std::vector<double>* normalized_stiffness = nullptr);
}
