#pragma once
#include "GuidedPlateStudy.h"
#include "lib_src/collision/Q4ContactBounds.h"
#include <cmath>

namespace crash::case_data::study_detail {
namespace ct=tlfea::contact;
namespace bounds=ct::q4_bounds;
namespace shell=tl::fea::reissner;
inline bool Nonnegative(double x) { return std::isfinite(x)&&x>=0; }
inline bool Reject(std::string& error,const char* text) { error=text; return false; }
bool ValidConfig(const GuidedStudyConfig&);
bool Certified(const GuidedStudyCertificate&);
bool SameStamp(const tl::fea::NodalStamp&,const tl::fea::NodalStamp&);
bool TimeMatches(double time,double expected,double h,std::uint64_t epoch);
bool Endpoint(const GuidedStudyConfig&,const GuidedPlateMetrics&,std::string&);
bool Force(const ct::Q4PlanarContactDiagnostics&,GuidedStudyCertificate&);
bool Sample(const GuidedStudyConfig&,const GuidedPlateMetrics&,const GuidedPlateFrame&,
            GuidedStudySample&,bool& partial,bool& unequal,std::string&);
} // namespace crash::case_data::study_detail
