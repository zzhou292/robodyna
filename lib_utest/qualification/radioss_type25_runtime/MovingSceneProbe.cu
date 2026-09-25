// SPDX-License-Identifier: AGPL-3.0-or-later
// Diagnostic continuation only. Completion is not trajectory acceptance.
#include "MovingSceneRig.h"
#include "Reference.h"
#include <cstdio>
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <map>
#include <memory>
using namespace native_runtime_test;
namespace {
struct File {std::FILE* p=nullptr;explicit File(const std::string& path){p=std::fopen(path.c_str(),"wx");if(!p)throw std::runtime_error("Cannot create diagnostic output "+path);}~File(){if(p)std::fclose(p);}};
struct Maximum {
  double absolute=0,scaled=0,actual=0,expected=0;std::uint64_t epoch=0;std::size_t entry=0;
  void Observe(double a,double b,double floor,std::uint64_t step,std::size_t index) {
    if(!std::isfinite(a)||!std::isfinite(b))throw std::runtime_error("Nonfinite diagnostic operand");
    const double error=std::abs(a-b),ratio=error/(floor+2e-8*std::abs(b));absolute=std::max(absolute,error);
    if(ratio>scaled){scaled=ratio;actual=a;expected=b;epoch=step;entry=index;}
  }
};
std::array<double,15> Fields(const n::NativeContactRow& r) {
  const auto& h=r.history.normal;return {h.previous_penetration,h.previous_stiffness,h.staged_penetration,h.staged_stiffness,h.damping_half_force,
    r.history.previous_force.x,r.history.previous_force.y,r.history.previous_force.z,r.history.staged_force.x,r.history.staged_force.y,r.history.staged_force.z,
    r.penetration_auxiliary,r.penetration_offset,r.selection_metric[0],r.selection_metric[1]};
}
}
int main(int argc,char** argv) try {
  if(argc!=2)throw std::runtime_error("Expected fresh diagnostic output directory");
  const std::filesystem::path directory=argv[1];if(!std::filesystem::is_directory(directory))throw std::runtime_error("Output directory must exist");
  File trace((directory/"nodal.csv").string()),hist((directory/"history.csv").string());
  std::fprintf(trace.p,"epoch,node,axis,x_mm,native_x_mm,v_mm_s,native_v_mm_s,contact_n,native_contact_n,structural_n,native_structural_n,total_n,native_total_n,total_defined\n");
  std::fprintf(hist.p,"force_epoch,secondary,field,actual,expected,scaled_error\n");
  ReferenceReader reader(TYPE25_NATIVE_REFERENCE_FILE);std::vector<ExpectedFrame> expected;
  for(unsigned step=0;step<=1000;++step)expected.push_back(reader.Read(step));reader.Finish();
  Rig rig;rig.Initialize();State state=rig.Read();std::map<std::string,Maximum> maxima;
  std::size_t marker_differences=0,icont_differences=0,packet_differences=0,sentinel_differences=0;
  std::size_t active_steps=0,rebuilds=0;std::array<bool,18> prior{};std::array<std::vector<unsigned>,18> episodes;
  const double floors[]{2e-8,2e-3,2e-8,2e-3,2e-5,2e-5,2e-5,2e-5,2e-5,2e-5,2e-5,2e-8,2e-8,2e-8,2e-8};
  for(unsigned step=0;step<1000;++step) {
    const auto& ref=expected[step];const auto& next=expected[step+1];
    for(unsigned i=0;i<54;++i){maxima["position_mm"].Observe(state.x[i]/.001,ref.position[i],2e-8,step,i);
      maxima["velocity_mm_s"].Observe(state.v[i]/.001,ref.velocity[i],2e-5,step,i);}
    Attempt a;rig.BeginMaterials(a);std::array<double,54> before{},after{};std::array<double,18> kb{},ka{};
    rig.Force(a,before,kb);Check(rig.contact.AssembleAccepted(rig.owner,a.token,a.assembly));rig.Force(a,after,ka);
    n::runtime_qualification::Observation observation;
    if(!n::runtime_qualification::Access::Read(rig.contact,rig.owner,a.token,a.assembly,&observation))throw std::runtime_error("Unobservable successful native assembly");
    std::vector<int> secondary,main;for(const auto& occurrence:observation.occurrences)if(occurrence.secondary>0){secondary.push_back(occurrence.secondary);main.push_back(occurrence.selected.local_main);}
    std::vector<int> ns,ms;std::vector<std::uint32_t> ends;
    for(const auto& packet:ref.geometry_packets){ns.insert(ns.end(),packet.secondary.begin(),packet.secondary.end());ms.insert(ms.end(),packet.main.begin(),packet.main.end());ends.push_back(ns.size());}
    if(secondary!=ns||main!=ms||observation.cohort_ends!=ends)++packet_differences;
    const auto diag=rig.contact.last_diagnostics();active_steps+=diag.active_forces!=0;rebuilds+=diag.reference_rebuilt;
    for(unsigned node=0;node<18;++node)for(unsigned axis=0;axis<3;++axis) {
      const auto i=3*node+axis;const double contact=after[i]-before[i],native_contact=ref.outgoing_force[i]-ref.incoming_force[i];
      maxima["contact_force_n"].Observe(contact,native_contact,2e-5,step,i);
      const bool available=!(rig.fixture.fixed[node]&(1u<<axis));
      const double total=available?observed::ObservedMass[node]*(next.velocity[i]-ref.velocity[i])/next.kick:0;
      const double structural=available?total-native_contact:0;
      if(available){maxima["derived_total_force_n"].Observe(after[i],total,2e-5,step,i);maxima["derived_structural_force_n"].Observe(before[i],structural,2e-5,step,i);}
      std::fprintf(trace.p,"%u,%llu,%u,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%u\n",step,
        static_cast<unsigned long long>(observed::NodeIds[node]),axis,state.x[i]/.001,ref.position[i],state.v[i]/.001,ref.velocity[i],
        contact,native_contact,before[i],structural,after[i],total,unsigned(available));
    }
    for(unsigned node=0;node<18;++node)maxima["contact_stiffness_n_per_mm"].Observe((ka[node]-kb[node])/1000,
      ref.outgoing_stiffness[node]-ref.incoming_stiffness[node],2e-3,step,node);
    rig.Prepare(a);Check(rig.Commit(a));state=rig.Read();
    if(state.stamp.epoch!=step+1||state.contact.force_base_stamp.epoch!=step)throw std::runtime_error("Native physical/history phase mismatch");
    for(unsigned row=0;row<18;++row) {
      const auto& actual=state.history[row].row;const auto& wanted=ref.rows[row];
      for(unsigned k=0;k<4;++k)if(actual.irtlm[k]!=wanted.irtlm[k]){++marker_differences;std::fprintf(hist.p,"%u,%u,irtlm%u,%d,%d,0\n",step,row+1,k,actual.irtlm[k],wanted.irtlm[k]);}
      if(state.initial_contact[row]!=ref.initial_contact[row])++icont_differences;
      const auto x=Fields(actual),y=Fields(wanted);
      for(unsigned k=0;k<x.size();++k) {
        if(k>=13&&std::abs(y[k])==1e20){if(x[k]!=y[k])++sentinel_differences;}
        maxima["history_"+std::to_string(k)].Observe(x[k],y[k],floors[k],step,row);
        const double ratio=std::abs(x[k]-y[k])/(floors[k]+2e-8*std::abs(y[k]));
        if(ratio>1)std::fprintf(hist.p,"%u,%u,%u,%.17g,%.17g,%.17g\n",step,row+1,k,x[k],y[k],ratio);
      }
      const bool active=actual.irtlm[0]!=0;if(active&&!prior[row])episodes[row].push_back(step);prior[row]=active;
    }
    if(step%100==0){std::fflush(trace.p);std::fflush(hist.p);std::printf("completed physical interval %u\n",step+1);std::fflush(stdout);}
  }
  for(unsigned i=0;i<54;++i){maxima["position_mm"].Observe(state.x[i]/.001,expected[1000].position[i],2e-8,1000,i);
    maxima["velocity_mm_s"].Observe(state.v[i]/.001,expected[1000].velocity[i],2e-5,1000,i);}
  File summary((directory/"summary.json").string());
  std::fprintf(summary.p,"{\"scope\":\"diagnostic continuation; not numerical acceptance\",\"physical_intervals\":1000,\"active_steps\":%zu,\"reference_rebuilds\":%zu,\"marker_differences\":%zu,\"icont_differences\":%zu,\"packet_differences\":%zu,\"sentinel_differences\":%zu,\"maxima\":{",active_steps,rebuilds,marker_differences,icont_differences,packet_differences,sentinel_differences);
  bool first=true;for(const auto& item:maxima){const auto& m=item.second;
    std::fprintf(summary.p,"%s\"%s\":{\"absolute\":%.17g,\"scaled\":%.17g,\"at_epoch\":%llu,\"entry\":%zu,\"actual\":%.17g,\"expected\":%.17g}",first?"":",",item.first.c_str(),m.absolute,m.scaled,static_cast<unsigned long long>(m.epoch),m.entry,m.actual,m.expected);first=false;}
  std::fprintf(summary.p,"},\"episodes\":[");for(unsigned i=0;i<18;++i){if(i)std::fputc(',',summary.p);std::fputc('[',summary.p);
    for(std::size_t j=0;j<episodes[i].size();++j)std::fprintf(summary.p,"%s%u",j?",":"",episodes[i][j]);std::fputc(']',summary.p);}std::fprintf(summary.p,"]}\n");
  return 0;
} catch(const std::exception& e){std::fprintf(stderr,"diagnostic failed: %s\n",e.what());return 1;}
