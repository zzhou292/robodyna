#pragma once
#include <Eigen/Core>
#include <string>

namespace tl::qualification::qeph::recurrence {
constexpr unsigned MaximumPowerGramCount=32769;
// P maps the initial state to the endpoint after count transitions. G sums
// states BEFORE those transitions: k=0..count-1 (P_k)^T P_k. In particular,
// count=0 means P=I,G=0. These are numerical values, not authenticated records.
struct PowerGramBlock {
  Eigen::MatrixXd power,gram;
  unsigned count=0;
};
// Same binary arithmetic as the original PowerGram; accepts count=0 as well.
bool BuildPowerGramBlock(const Eigen::MatrixXd&,unsigned count,
                         PowerGramBlock&,std::string&);
// Chronological first THEN second: P=P_second*P_first,
// G=G_first+P_first^T*G_second*P_first. No repeated junction sample.
bool ComposePowerGramBlocks(const PowerGramBlock& first,
                            const PowerGramBlock& second,
                            PowerGramBlock&,std::string&);
// Add the final endpoint ONCE, giving count+1 samples. No spectral or
// symmetrization policy is imposed here; callers retain/check raw matrices.
bool EndpointInclusiveGram(const PowerGramBlock&,Eigen::MatrixXd&,std::string&);
// All operations stage publication and permit output to alias an input.
// Dimensions are 1..194; total transition count is bounded by 32769.
} // namespace tl::qualification::qeph::recurrence
