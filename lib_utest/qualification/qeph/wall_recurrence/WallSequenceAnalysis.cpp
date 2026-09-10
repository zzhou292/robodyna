#include "WallRecurrenceSpectrum.h"
#include <utility>

namespace tl::qualification::qeph::wall_recurrence {
namespace {
bool Accumulate(const std::vector<WallOperatorRun>& runs,const Eigen::VectorXd& diagonal,
                bool weighted,recurrence::PowerGramBlock& out,std::string& error) {
  recurrence::PowerGramBlock result;
  const auto identity=Eigen::MatrixXd::Identity(diagonal.size(),diagonal.size()).eval();
  if(!recurrence::BuildPowerGramBlock(identity,0,result,error)) return false;
  for(const auto& run:runs) {
    Eigen::MatrixXd transformed;
    if(weighted&&!ApplyWallStateMetric(*run.matrix,diagonal,transformed,error)) return false;
    recurrence::PowerGramBlock next;
    if(!recurrence::BuildPowerGramBlock(weighted?transformed:*run.matrix,run.count,next,error)||
       !recurrence::ComposePowerGramBlocks(result,next,result,error)) return false;
  }
  out=std::move(result); return true;
}
}
WallSequenceAnalysis AnalyzeWallSequence(const std::vector<WallOperatorRun>& runs,
                                        const Eigen::VectorXd& diagonal) {
  WallSequenceAnalysis out;
  if(runs.empty()||runs.size()>64||!ValidWallStateDiagonal(diagonal)) {
    out.diagnostic="Invalid chronological run count/metric"; return out;
  }
  unsigned total=0;
  for(const auto& run:runs) {
    if(!run.matrix||run.matrix->rows()!=diagonal.size()||run.matrix->cols()!=diagonal.size()||
       !run.matrix->allFinite()||run.count>32768-total) {
      out.diagnostic="Invalid chronological operator/dimension/count"; return out;
    }
    total+=run.count;
  }
  recurrence::PowerGramBlock raw,weighted;
  if(Accumulate(runs,diagonal,false,raw,out.diagnostic)) out.raw=AnalyzeWallGram(raw);
  else return out;
  if(Accumulate(runs,diagonal,true,weighted,out.diagnostic)) out.weighted=AnalyzeWallGram(weighted);
  else return out;
  out.complete=out.raw.complete&&out.weighted.complete;
  // Raw gain is diagnostic. A finite large raw drift must not tune D or delete
  // coordinates; only the predeclared weighted mean carries the gain limit.
  out.passed=out.complete&&out.weighted.within_gain_budget;
  if(!out.passed) out.diagnostic="A full-state raw/weighted Gram is unresolved or weighted gain failed";
  return out;
}
} // namespace tl::qualification::qeph::wall_recurrence
