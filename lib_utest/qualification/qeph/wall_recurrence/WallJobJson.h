#pragma once
#include "WallJobReport.h"
#include "WallRawJson.h"

namespace tl::qualification::qeph::wall_recurrence::job_json {
namespace io=crash::output;
void Scalar(io::Document&,const char* name,double,bool partial);
void Status(io::Document&,bool complete,bool passed,const std::string& diagnostic);
io::Document Baseline(const WallBaselineAnalysis&,unsigned dimension);
io::Document Identities(const WallIdentityAnalysis&,unsigned dimension);
io::Document Spectrum(const WallSpectrum&,unsigned dimension);
io::Document Gram(const WallGramAnalysis&,unsigned dimension);
io::Document Sequence(const WallSequenceAnalysis&,unsigned dimension);
io::Document Difference(const WallDifference&);
io::Document Comparison(const WallAnalysisComparison&);
io::Document Context(const WallBranchAnalysis&,unsigned dimension);
io::Document Amplitude(const WallAmplitudeAnalysis&,const RawJob&,unsigned step,unsigned amplitude);
io::Document Contact(const WallContactRecheck&,unsigned dimension,unsigned nodes);
io::Document Step(const WallStepAnalysis&);
} // namespace tl::qualification::qeph::wall_recurrence::job_json
