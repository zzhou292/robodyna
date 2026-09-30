#include "WallJobJson.h"

namespace tl::qualification::qeph::wall_recurrence::job_json {
namespace {
bool SameWindow(const WallSwitchingWindow& a,const WallSwitchingWindow& b) {
  return a.entry_shift==b.entry_shift&&a.exit_shift==b.exit_shift&&a.entry_base_epoch==b.entry_base_epoch&&
    a.exit_base_epoch==b.exit_base_epoch&&a.inactive_before==b.inactive_before&&a.active==b.active&&a.inactive_after==b.inactive_after;
}
io::Document MatrixDescriptor(const Eigen::MatrixXd& matrix,unsigned dimension,bool required) {
  io::Require((matrix.size()==0&&!required)||(matrix.rows()==dimension&&matrix.cols()==dimension),"Invalid derived matrix dimensions");
  io::Require(!required||matrix.allFinite(),"Nonfinite completed derived operator");
  io::Document d; d.SetObject(); io::Integer(d,"rows",matrix.rows()); io::Integer(d,"columns",matrix.cols());
  io::Boolean(d,"present",matrix.size()!=0); io::Boolean(d,"all_finite",matrix.allFinite());
  io::String(d,"sha256",WallDerivedMatrixHash(matrix));
  io::String(d,"hash_encoding","EncodeRawJson(raw_detail::Matrix): rows,columns,all_finite,values_rows; partial marker bits retained"); return d;
}
}
io::Document Amplitude(const WallAmplitudeAnalysis& a,const RawJob& raw,unsigned step,unsigned amplitude) {
  const auto dimension=static_cast<unsigned>(raw.model.native().dictionary.size());
  io::Require(step<6&&amplitude<3&&a.amplitude==recurrence::Amplitudes[amplitude]&&(!a.input_complete||a.attempted)&&
    (!a.complete||a.input_complete),"Invalid derived amplitude identity/flags");
  io::Document d; d.SetObject(); io::Number(d,"amplitude",a.amplitude); io::Boolean(d,"attempted",a.attempted);
  io::Boolean(d,"input_complete",a.input_complete); Status(d,a.complete,a.passed,a.diagnostic);
  raw_detail::Field(d,"moving_baseline",Baseline(a.baseline,dimension)); const auto& b=a.branches;
  io::Require((b.h==0&&!a.input_complete)||b.h==raw.steps[step].h,"Changed derived branch step");
  io::Require(b.completed_branches<=2&&b.completed_events<=9,"Invalid derived completion counts");
  io::Require(!a.complete||(a.baseline.complete&&b.complete),"Completed amplitude lacks complete baseline/branch evidence");
  io::Require(!a.passed||(a.baseline.passed&&b.passed),"Passed amplitude lacks passing component evidence");
  io::Require(!b.complete||(b.completed_branches==2&&b.completed_events==9),"Completed branch analysis has incomplete counts");
  io::Document branches; branches.SetObject(); Status(branches,b.complete,b.passed,b.diagnostic);
  io::Integer(branches,"completed_branches",b.completed_branches); io::Integer(branches,"completed_events",b.completed_events);
  io::String(branches,"full_operator_recipe","A_branch=(retained_native_shell * BuildContactKick(model,h,branch)).eval()");
  io::String(branches,"kick_entry_recipe","Identity; active B[v_ix,x_ix]=-double(long_double(h)*k_i/m_i*x_scale/v_scale), left-associated");
  io::String(branches,"weighted_operator_recipe","ApplyWallStateMetric: weighted(row,col)=D[row]*A_branch(row,col)/D[col], binary64 left-associated");
  io::String(branches,"matrix_source","Exact native payload hash in header; model/dictionary in authenticated raw model; D in context file");
  io::Value constants(rapidjson::kArrayType),events(rapidjson::kArrayType);
  for(unsigned i=0;i<2;++i) {
    const auto& c=b.branches[i];
    io::Require(!b.complete||(c.identities.complete&&c.spectrum.complete&&c.weighted_spectrum.complete&&c.continuous.complete),
      "Completed branch analysis lacks complete component evidence");
    io::Require(!b.passed||(c.identities.passed&&c.spectrum.passed&&c.weighted_spectrum.passed&&c.continuous.passed),
      "Passed branch analysis lacks passing component evidence");
    // An unavailable default branch still has its default enum. Index is the
    // frozen role until BuildContactBranch has produced an actual matrix.
    io::Require(c.full.size()==0||c.branch==(i?ContactBranch::Active:ContactBranch::Inactive),"Changed derived branch role");
    io::Document v; v.SetObject(); io::String(v,"branch",i?"active":"inactive");
    raw_detail::Field(v,"full_operator",MatrixDescriptor(c.full,dimension,b.complete));
    raw_detail::Field(v,"weighted_operator",MatrixDescriptor(c.weighted,dimension,b.complete));
    raw_detail::Field(v,"identities",Identities(c.identities,dimension));
    raw_detail::Field(v,"raw_spectrum",Spectrum(c.spectrum,dimension));
    raw_detail::Field(v,"weighted_spectrum",Spectrum(c.weighted_spectrum,dimension));
    raw_detail::Field(v,"continuous_gram",Sequence(c.continuous,dimension)); raw_detail::Append(branches,constants,v);
  }
  for(unsigned i=0;i<9;++i) {
    io::Require(!b.complete||b.events[i].sequence.complete,"Completed branch analysis lacks event evidence");
    io::Require(!b.passed||b.events[i].sequence.passed,"Passed branch analysis lacks passing event evidence");
    io::Document event; event.SetObject(); io::Integer(event,"context_window_index",i);
    const bool available=b.events[i].window.entry_base_epoch!=0||b.events[i].window.exit_base_epoch!=0;
    io::Require(!available||SameWindow(b.events[i].window,b.schedule.windows[i]),"Derived event window differs from saved context");
    io::Require(!b.events[i].sequence.complete||available,"Completed event lacks its actual window");
    io::Boolean(event,"window_available",available);
    raw_detail::Field(event,"gram",Sequence(b.events[i].sequence,dimension)); raw_detail::Append(branches,events,event);
  }
  branches.AddMember("constant_branches",constants,branches.GetAllocator()); branches.AddMember("events",events,branches.GetAllocator());
  raw_detail::Field(d,"branches",branches); return d;
}
} // namespace tl::qualification::qeph::wall_recurrence::job_json
