#pragma once
#include "WallDerivedRead.h"
#include "WallRawReadFields.h"
#include "WallJobJson.h"
#include <map>

namespace tl::qualification::qeph::wall_recurrence::derived_read {
namespace io=crash::output;
namespace rd=read_detail;
struct State {
  const RawReadResult& raw;
  const WallDerivedReadBinding& binding;
  WallDerivedReceipt receipt;
  WallJobAnalysis analysis;
  std::array<WallBranchAnalysis,6> contexts;
  std::array<bool,6> context_seen{},context_usable{},step_seen{};
  std::array<std::array<bool,3>,6> amplitude_seen{};
  std::array<std::array<bool,2>,6> contact_seen{};
  std::map<std::string,std::string> hashes;
  std::string origin;
  std::size_t producer_budget=0;
  unsigned next_step=0;
  bool progress_without_job=true;
};
std::string RawHash(const RawReadResult&,const std::string&);
void RawLink(const io::Value&,const RawReadResult&,const std::string&);
void Header(const io::Document&,const char*,const State&);
void Inventory(const io::Document&,const State&,bool final);
void Context(const io::Document&,unsigned,State&);
void Amplitude(const io::Document&,unsigned,unsigned,State&);
void Contact(const io::Document&,unsigned,unsigned,State&);
void Step(const io::Document&,unsigned,State&);
void ReleaseDense(WallStepAnalysis&);
// Parse measured numerical evidence, then reproduce its original scalar
// predicates and canonical shape. Diagnostics remain producer text.
WallSpectrum Spectrum(const io::Value&,const Eigen::MatrixXd&,unsigned);
WallGramAnalysis Gram(const io::Value&,unsigned dimension,unsigned ordinary_steps);
WallSequenceAnalysis Sequence(const io::Value&,unsigned dimension,unsigned ordinary_steps);
inline std::string Diagnostic(const io::Value& v) { return rd::Text(rd::Field(v,"diagnostic")); }
inline void Same(const io::Value& actual,const io::Value& expected,const char* message) {
  io::Require(rd::Same(actual,expected),message);
}
} // namespace tl::qualification::qeph::wall_recurrence::derived_read
