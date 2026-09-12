#pragma once
#include "SourceAssemblyData.h"
#include <utility>
namespace crash::modelio::assembly::reader {
// Authenticated source evidence is supplied by the existing source-block reader.
// This shared decoder preserves exact scaled bits and checks repeated ownership.
void ReadSourceCurve(const SourceBlock&,const std::vector<std::pair<std::size_t,std::string>>& cards,
    SourceId id,std::size_t count,double ordinate_scale,std::vector<double>& x,std::vector<double>& y);
} // namespace crash::modelio::assembly::reader
