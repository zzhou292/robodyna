#pragma once
#include "RecurrenceAudit.h"
namespace tl::qualification::qeph::recurrence {
bool CheckStructuralIdentities(const Model&,double h,const Eigen::MatrixXd&,MapAnalysis&);
}
