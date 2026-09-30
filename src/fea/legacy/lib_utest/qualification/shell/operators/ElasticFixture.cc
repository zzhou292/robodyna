#include "ElasticFixture.h"

namespace shell_spike {
Attempt EvaluateElastic(const ElasticInput& in,ThicknessRule rule,
                        ElasticResult* committed,const Options& options) {
  switch(rule) {
    case ThicknessRule::OriginalMidpoint3:
      return detail::EvaluateElasticOriginal(in,committed,options);
    case ThicknessRule::Gauss3:
      return detail::EvaluateElasticGauss(in,committed,options);
    default:
      return {Status::InvalidInput,"Unsupported thickness rule",0};
  }
}
}  // namespace shell_spike
