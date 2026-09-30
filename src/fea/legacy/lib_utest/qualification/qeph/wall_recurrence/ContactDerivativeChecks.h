#pragma once
#include "lib_utest/qualification/qeph/free_response/RecurrenceAudit.h"
#include "lib_src/collision/Q4ContactBounds.h"
#include <algorithm>
#include <cmath>

namespace tl::qualification::qeph::wall_recurrence::derivative_detail {
namespace r=recurrence;
namespace contact=tlfea::contact;
namespace b=contact::q4_bounds;
// Extracted from the retained actual probe without changing operand grouping,
// interval operations or budgets. Callers validate finite matching dimensions.
inline bool CompareDirection(const Eigen::MatrixXd& a,const Eigen::VectorXd& direction,
                             const Eigen::VectorXd& quotient,double& residual,double& budget) {
  double norm_lower=0,direction_norm=0;
  residual=0;
  for(Eigen::Index column=0;column<a.cols();++column)
    direction_norm=std::max(direction_norm,std::abs(direction[column]));
  for(Eigen::Index row=0;row<a.rows();++row) {
    double row_lower=0;
    contact::Q4IntegralInterval dot{};
    for(Eigen::Index column=0;column<a.cols();++column) {
      if(!b::AddScalar(row_lower,std::abs(a(row,column)),false,&row_lower)) return false;
      contact::Q4IntegralInterval term;
      if(!b::MultiplyScalar(a(row,column),direction[column],false,&term.lower)||
         !b::MultiplyScalar(a(row,column),direction[column],true,&term.upper)||!b::Add(dot,term,&dot)) return false;
    }
    norm_lower=std::max(norm_lower,row_lower);
    double from_lower=0,from_upper=0;
    if(!b::AbsoluteDifferenceUpper(quotient[row],dot.lower,&from_lower)||
       !b::AbsoluteDifferenceUpper(quotient[row],dot.upper,&from_upper)) return false;
    residual=std::max(residual,std::max(from_lower,from_upper));
  }
  double product=0;
  return b::MultiplyScalar(norm_lower,direction_norm,false,&product)&&
         b::MultiplyScalar(r::MatrixTolerance,std::max(1.,product),false,&budget);
}
// Matching dimensions and positive epsilon are caller preconditions. Preserve
// even a nonfinite computed quotient as evidence; its caller decides validity.
inline Eigen::VectorXd OneSidedQuotient(const Eigen::VectorXd& baseline,
                                      const Eigen::VectorXd& coarse,const Eigen::VectorXd& fine,double epsilon) {
  Eigen::VectorXd quotient(baseline.size());
  for(Eigen::Index row=0;row<quotient.size();++row) {
    const long double difference=4.L*fine[row]-coarse[row]-3.L*baseline[row];
    quotient[row]=static_cast<double>(difference/epsilon);
  }
  return quotient;
}
} // namespace tl::qualification::qeph::wall_recurrence::derivative_detail
