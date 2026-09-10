#include "WallRecurrenceAnalysis.h"
#include <algorithm>
#include <cmath>

namespace tl::qualification::qeph::wall_recurrence {
namespace bounds=contact::q4_bounds;
namespace {
bool SameWindow(const WallSwitchingWindow& a,const WallSwitchingWindow& b) {
  return a.entry_shift==b.entry_shift&&a.exit_shift==b.exit_shift&&
    a.entry_base_epoch==b.entry_base_epoch&&a.exit_base_epoch==b.exit_base_epoch&&
    a.inactive_before==b.inactive_before&&a.active==b.active&&a.inactive_after==b.inactive_after;
}
}
WallDifference CompareWallMatrices(const Eigen::MatrixXd& a,const Eigen::MatrixXd& b) {
  if(a.rows()<1||a.rows()>194||a.rows()!=a.cols()||a.rows()!=b.rows()||a.cols()!=b.cols()||
     !a.allFinite()||!b.allFinite()) return {};
  WallDifference out; double scale=1;
  for(Eigen::Index row=0;row<a.rows();++row) for(Eigen::Index col=0;col<a.cols();++col) {
    double difference=0;
    if(!bounds::AbsoluteDifferenceUpper(a(row,col),b(row,col),&difference)) return out;
    out.difference=std::max(out.difference,difference);
    scale=std::max(scale,std::max(std::abs(a(row,col)),std::abs(b(row,col))));
  }
  out.complete=bounds::MultiplyScalar(recurrence::MatrixTolerance,scale,false,&out.budget)&&out.budget>0;
  out.passed=out.complete&&out.difference<=out.budget; return out;
}
WallDifference CompareWallGains(double a,double b) {
  if(!std::isfinite(a)||!std::isfinite(b)||a<0||b<0) return {};
  WallDifference out; double relative=0;
  out.complete=bounds::AbsoluteDifferenceUpper(a,b,&out.difference)&&
    bounds::MultiplyScalar(WallGainRelativeTolerance,std::max(a,b),false,&relative)&&
    bounds::AddScalar(relative,WallGainAbsoluteTolerance,false,&out.budget)&&out.budget>0;
  out.passed=out.complete&&out.difference<=out.budget; return out;
}
WallAnalysisComparison CompareWallAnalyses(const WallBranchAnalysis& a,const WallBranchAnalysis& b) {
  WallAnalysisComparison out;
  if(!a.complete||!b.complete||!FrozenStep(a.h)||a.h!=b.h||
     !ValidWallStateDiagonal(a.metric.diagonal)||a.metric.diagonal.size()!=b.metric.diagonal.size()||
     a.metric.diagonal!=b.metric.diagonal||a.schedule.ordinary_steps!=b.schedule.ordinary_steps||
     a.schedule.total_steps!=b.schedule.total_steps) {
    out.diagnostic="Incompatible or incomplete full-state analyses"; return out;
  }
  for(unsigned i=0;i<9;++i) if(!SameWindow(a.events[i].window,b.events[i].window)) {
    out.diagnostic="Different frozen event windows"; return out;
  }
  bool complete=true,passed=true;
  for(unsigned i=0;i<2;++i) {
    out.matrices[i]=CompareWallMatrices(a.branches[i].full,b.branches[i].full);
    complete=complete&&out.matrices[i].complete; passed=passed&&out.matrices[i].passed;
  }
  for(unsigned i=0;i<11;++i) {
    const auto& x=i<2?a.branches[i].continuous:a.events[i-2].sequence;
    const auto& y=i<2?b.branches[i].continuous:b.events[i-2].sequence;
    out.raw_gains[i]=CompareWallGains(x.raw.mean_gain,y.raw.mean_gain);
    out.weighted_gains[i]=CompareWallGains(x.weighted.mean_gain,y.weighted.mean_gain);
    complete=complete&&x.complete&&y.complete&&out.raw_gains[i].complete&&out.weighted_gains[i].complete;
    passed=passed&&out.weighted_gains[i].passed;
  }
  out.complete=complete; out.passed=complete&&passed;
  if(!out.passed) out.diagnostic="Frozen amplitude/boost matrix or gain consistency failed";
  return out;
}
double SelectWallScreenStep(const std::array<bool,6>& pass) noexcept {
  if(!pass[0]||!pass[1]||!pass[2]||!pass[3]) return 0;
  return pass[4]?recurrence::H0:recurrence::H0/2;
}
} // namespace tl::qualification::qeph::wall_recurrence
