#include "RecurrenceReport.h"
#include "chrono_thirdparty/rapidjson/stringbuffer.h"
#include "chrono_thirdparty/rapidjson/writer.h"
#include <cmath>

namespace tl::qualification::qeph::recurrence {
namespace io=crash::output;
namespace {
void Push(io::Document& d,io::Value& a,const io::Document& child) {
  io::Value value; value.CopyFrom(child,d.GetAllocator()); a.PushBack(value,d.GetAllocator());
}
io::Document CoordinateReport(const Coordinate& c) {
  static const char* groups[]{"position","orientation_tangent","velocity","spin","history","force_cache"};
  io::Require(static_cast<unsigned>(c.group)<6&&std::isfinite(c.scale)&&c.scale>0,"Invalid dictionary record");
  io::Document d; d.SetObject(); io::String(d,"group",groups[static_cast<unsigned>(c.group)]);
  io::String(d,"name",c.name); io::String(d,"unit",c.unit); io::Number(d,"scale",c.scale);
  io::Integer(d,"entity",c.entity); io::Integer(d,"component",c.component); io::Boolean(d,"feedback",c.feedback); return d;
}
io::Document AnalysisReport(const MapAnalysis& a) {
  io::Document d; d.SetObject(); io::Boolean(d,"complete",a.complete); io::Boolean(d,"passed",a.passed);
  io::String(d,"diagnostic",a.diagnostic);
  io::Number(d,"schur_residual",a.schur_residual); io::Number(d,"orthogonality_error",a.orthogonality_error);
  io::Number(d,"spectral_radius",a.spectral_radius); io::Number(d,"gram_mean_gain",a.gram_gain);
  io::Number(d,"gram_antisymmetry",a.gram_antisymmetry); io::Number(d,"zero_feedback_error",a.zero_feedback_error);
  io::Number(d,"observer_error",a.observer_error); io::Number(d,"rigid_error",a.rigid_error);
  io::Number(d,"rigid_basis_condition",a.rigid_basis_condition); io::Number(d,"rigid_projection_residual",a.rigid_projection_residual);
  io::Integer(d,"near_one_count",a.near_one); io::Integer(d,"near_zero_count",a.near_zero);
  io::Require(a.eigenvalues.size()<=124,"Unbounded feedback spectrum");
  io::Value eigen(rapidjson::kArrayType);
  for(auto z:a.eigenvalues) { const double parts[]{z.real(),z.imag()}; eigen.PushBack(io::FiniteArray(d,parts,2),d.GetAllocator()); }
  d.AddMember("eigenvalues_real_imag",eigen,d.GetAllocator()); return d;
}
io::Document ProbeReport(const MatrixProbe& p,unsigned dimension,bool raw) {
  io::Require(p.full.rows()==p.full.cols()&&p.full.rows()<=194&&p.full.allFinite()&&p.completed_columns<=dimension&&
    (p.full.rows()==dimension||(!p.complete&&p.full.rows()==0&&p.completed_columns==0)),"Invalid raw probe shape/values");
  io::Require(!p.complete||p.completed_columns==dimension,"Completed probe has missing columns");
  io::Document d; d.SetObject(); io::Boolean(d,"complete",p.complete); io::Integer(d,"completed_columns",p.completed_columns);
  io::Number(d,"amplitude",p.amplitude); io::String(d,"diagnostic",p.diagnostic); io::Integer(d,"dimension",dimension);
  if(raw) {
    io::Value matrix(rapidjson::kArrayType);
    for(Eigen::Index r=0;r<p.full.rows();++r) {
      io::Value row(rapidjson::kArrayType);
      for(Eigen::Index c=0;c<p.full.cols();++c) row.PushBack(p.full(r,c),d.GetAllocator());
      matrix.PushBack(row,d.GetAllocator());
    }
    d.AddMember("full_matrix_rows",matrix,d.GetAllocator());
  }
  return d;
}
}
io::Document DescribeAudit(const Audit& result,const io::Document& provenance,bool raw,const std::string& hash,std::size_t bytes) {
  io::Require(provenance.IsObject()&&!provenance.ObjectEmpty(),"Missing bounded audit provenance binding");
  io::Require((result.selected_h==0||result.selected_h==H0||result.selected_h==H0/2)&&
    result.audit_passed==(result.selected_h>0),"Inconsistent audit selection");
  if(!raw) io::Require(hash.size()==64&&hash.find_first_not_of("0123456789abcdef")==std::string::npos&&bytes>0&&bytes<=32u*1024*1024,
                      "Missing exact raw report hash/byte binding");
  io::Document d; d.SetObject(); io::String(d,"schema","robo-dyna-qeph-recurrence-audit-v1");
  io::String(d,"phase",raw?"raw-probes":"decision"); io::Boolean(d,"audit_passed",result.audit_passed);
  io::Boolean(d,"simulation_ready",false); io::Boolean(d,"trajectory_execution_qualified",false);
  io::String(d,"map_scope","Native centered LAW1 QEPH full kick, drift, material/history and next force cache; ITHK0 ISROT0 IDRIL0, DM=DN=.015");
  io::String(d,"probe_phase","Prescribed base epoch1/timeh to endpoint epoch2/time2h; no runtime initialization or owner");
  io::String(d,"gram_scope","Mean finite-horizon gain in declared nondimensional feedback norm; not a uniform gain or general CFL theorem");
  io::Number(d,"selected_candidate_dt_s",result.selected_h); io::Number(d,"horizon_s",4096*H0);
  io::Number(d,"matrix_tolerance",MatrixTolerance); io::Number(d,"decomposition_tolerance",DecompositionTolerance);
  io::Number(d,"maximum_gram_mean_gain",MaximumGramGain);
  io::FiniteArray(d,"fixed_dt_grid_s",Steps.data(),Steps.size()); io::FiniteArray(d,"fixed_amplitudes",Amplitudes.data(),Amplitudes.size());
  io::Value bound; bound.CopyFrom(provenance,d.GetAllocator()); d.AddMember("provenance",bound,d.GetAllocator());
  if(!raw) { io::String(d,"raw_report_name","raw-matrices.json"); io::String(d,"raw_report_sha256",hash); io::Integer(d,"raw_report_bytes",bytes); }
  io::Value cases(rapidjson::kArrayType);
  for(unsigned c=0;c<result.cases.size();++c) {
    const auto& fixture=result.cases[c]; const unsigned dimension=static_cast<unsigned>(fixture.dictionary.size());
    io::Require(fixture.elements==c+1&&dimension<=194&&(!fixture.complete||
      (fixture.nodes==2*(c+2)&&dimension==12*fixture.nodes+61*fixture.elements)),"Invalid fixture dictionary cardinality");
    io::Document cd; cd.SetObject(); io::Integer(cd,"elements",fixture.elements); io::Integer(cd,"nodes",fixture.nodes);
    io::Boolean(cd,"complete",fixture.complete); io::String(cd,"diagnostic",fixture.diagnostic);
    io::Value dictionary(rapidjson::kArrayType);
    for(const auto& coordinate:fixture.dictionary) Push(cd,dictionary,CoordinateReport(coordinate));
    cd.AddMember("dictionary",dictionary,cd.GetAllocator()); io::Value steps(rapidjson::kArrayType);
    for(unsigned s=0;s<Steps.size();++s) {
      const auto& step=fixture.steps[s];
      io::Require(!fixture.complete||step.h==Steps[s],"Frozen grid identity changed");
      io::Document sd; sd.SetObject(); io::Number(sd,"fixed_dt_s",step.h); io::Boolean(sd,"passed",step.passed);
      io::String(sd,"diagnostic",step.diagnostic);
      io::FiniteArray(sd,"matrix_difference_max_abs",step.matrix_differences.data(),2);
      io::FiniteArray(sd,"gram_gain_relative_difference",step.gram_relative_differences.data(),2);
      io::Value probes(rapidjson::kArrayType),analyses(rapidjson::kArrayType);
      for(unsigned a=0;a<Amplitudes.size();++a) {
        io::Require(!fixture.complete||step.probes[a].amplitude==Amplitudes[a],"Frozen amplitude changed");
        if(result.audit_passed&&s<=(result.selected_h==H0?4u:3u))
          io::Require(fixture.complete&&step.passed&&step.probes[a].complete&&step.analysis[a].passed&&step.analysis[a].complete,
                      "Selected candidate depends on an unqualified matrix");
        Push(sd,probes,ProbeReport(step.probes[a],dimension,raw)); Push(sd,analyses,AnalysisReport(step.analysis[a]));
      }
      sd.AddMember("probes",probes,sd.GetAllocator()); sd.AddMember("analyses",analyses,sd.GetAllocator()); Push(cd,steps,sd);
    }
    cd.AddMember("steps",steps,cd.GetAllocator()); Push(d,cases,cd);
  }
  d.AddMember("cases",cases,d.GetAllocator()); return d;
}
std::string EncodeReport(const io::Document& report) {
  rapidjson::StringBuffer buffer; rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
  io::Require(report.Accept(writer),"Audit report serialization failed");
  io::Require(buffer.GetSize()<=32u*1024*1024,"Audit report exceeds 32 MiB cap");
  return std::string(buffer.GetString(),buffer.GetSize());
}
} // namespace tl::qualification::qeph::recurrence
