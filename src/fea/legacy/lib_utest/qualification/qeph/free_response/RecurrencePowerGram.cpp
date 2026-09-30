#include "RecurrencePowerGram.h"
#include <utility>

namespace tl::qualification::qeph::recurrence {
namespace {
bool Square(const Eigen::MatrixXd& a) {
  return a.rows()>=1&&a.rows()<=194&&a.rows()==a.cols()&&a.allFinite();
}
bool Valid(const PowerGramBlock& b) {
  if(!Square(b.power)||!Square(b.gram)||b.power.rows()!=b.gram.rows()||
     b.count>MaximumPowerGramCount) return false;
  if(b.count==0) {
    return b.power==Eigen::MatrixXd::Identity(b.power.rows(),b.power.cols())&&
           b.gram==Eigen::MatrixXd::Zero(b.gram.rows(),b.gram.cols());
  }
  return true;
}
}
bool BuildPowerGramBlock(const Eigen::MatrixXd& a,unsigned count,
                         PowerGramBlock& output,std::string& error) {
  if(!Square(a)||count>MaximumPowerGramCount) {
    error="Invalid finite-horizon Gram dimensions/count"; return false;
  }
  const unsigned original_count=count;
  Eigen::MatrixXd p=Eigen::MatrixXd::Identity(a.rows(),a.cols()),g=Eigen::MatrixXd::Zero(a.rows(),a.cols());
  Eigen::MatrixXd block_p=a,block_g=p;
  // Keep the qualified PowerGram loop's expression grouping and order.
  while(count) {
    if(count&1u) {
      g=(g+p.transpose()*block_g*p).eval(); p=(block_p*p).eval();
      if(!p.allFinite()||!g.allFinite()) { error="Nonfinite accumulated power/Gram"; return false; }
    }
    count>>=1u;
    if(count) {
      block_g=(block_g+block_p.transpose()*block_g*block_p).eval(); block_p=(block_p*block_p).eval();
      if(!block_p.allFinite()||!block_g.allFinite()) { error="Nonfinite binary power/Gram block"; return false; }
    }
  }
  PowerGramBlock result{std::move(p),std::move(g),original_count};
  output=std::move(result); error.clear(); return true;
}
bool ComposePowerGramBlocks(const PowerGramBlock& first,const PowerGramBlock& second,
                            PowerGramBlock& output,std::string& error) {
  if(!Valid(first)||!Valid(second)||first.power.rows()!=second.power.rows()||
     first.count>MaximumPowerGramCount-second.count) {
    error="Invalid chronological power/Gram blocks"; return false;
  }
  PowerGramBlock result;
  result.count=first.count+second.count;
  result.gram=(first.gram+first.power.transpose()*second.gram*first.power).eval();
  result.power=(second.power*first.power).eval();
  if(!result.power.allFinite()||!result.gram.allFinite()) {
    error="Nonfinite chronological power/Gram composition"; return false;
  }
  output=std::move(result); error.clear(); return true;
}
bool EndpointInclusiveGram(const PowerGramBlock& b,Eigen::MatrixXd& output,std::string& error) {
  if(!Valid(b)) { error="Invalid endpoint power/Gram block"; return false; }
  Eigen::MatrixXd result=(b.gram+b.power.transpose()*b.power).eval();
  if(!result.allFinite()) { error="Nonfinite endpoint-inclusive Gram"; return false; }
  output=std::move(result); error.clear(); return true;
}
} // namespace tl::qualification::qeph::recurrence
