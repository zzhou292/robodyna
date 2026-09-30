#include "WallResponseData.h"
#include "../response/ResponseFieldVisitor.h"
#include "lib_src/elements/qeph/QephStartup.h"
#include <cmath>
#include <cstring>
#include <utility>

namespace tl::qualification::qeph::wall_response {
bool ValidConfig(const Config& c) noexcept {
  if((c.cells!=1&&c.cells!=2)||(c.refinement!=1&&c.refinement!=2&&c.refinement!=4)||
     (c.selected_h!=H0&&c.selected_h!=H0/2)||c.screen_index_sha.size()!=64) return false;
  for(char x:c.screen_index_sha) if(!((x>='0'&&x<='9')||(x>='a'&&x<='f'))) return false;
  return true;
}
double Step(const Config& c) noexcept { return ValidConfig(c)?c.selected_h/c.refinement:0; }
unsigned Steps(const Config& c) noexcept { const auto h=Step(c); return h?static_cast<unsigned>(Horizon/h):0; }
unsigned SampleStride(const Config& c) noexcept { return Steps(c)/256; }
bool BuildModel(unsigned cells,Model& output,std::string& error) {
  Model m;
  if(!wr::BuildWallRecurrenceModel(cells,m.screened_,error)) return false;
  const auto& native=m.screened_.native(); auto& f=m.fields_;
  f.cells=cells; f.nodes=native.nodes; f.mass=native.mass; f.inertia=native.inertia;
  f.connectivity=native.connectivity;
  for(unsigned e=0;e<cells;++e) {
    const auto& original=native.reference[e].data(); port::ReferenceInput in;
    for(unsigned i=0;i<4;++i) {
      in.node_ids[i]=original.input.node_ids[i]; const auto x=original.input.position[i];
      in.position[i]={x.x,x.y,x.z};
    }
    in.density=original.input.density; in.young_modulus=original.input.young_modulus;
    in.poisson_ratio=original.input.poisson_ratio; in.thickness=original.input.thickness;
    if(port::InitializeReference(in,f.reference[e])!=port::Status::kSuccess) { error="Wall response port reference rejected"; return false; }
    for(unsigned i=0;i<4;++i) {
      const auto n=f.connectivity[e][i]; const auto x=in.position[i];
      f.initial_position[3*n]=x.x; f.initial_position[3*n+1]=x.y; f.initial_position[3*n+2]=x.z;
      f.physical[n]+=original.physical_inertia[i]; f.added[n]+=original.added_inertia[i];
    }
  }
  long double energy=0,momentum=0;
  for(unsigned n=0;n<f.nodes;++n) {
    energy+=.5L*f.mass[n]*wr::ImpactSpeed*wr::ImpactSpeed;
    momentum+=static_cast<long double>(f.mass[n])*wr::ImpactSpeed;
  }
  m.energy_=static_cast<double>(energy); m.momentum_=static_cast<double>(momentum);
  m.force_scale_=m.screened_.law().stiffness_per_area*m.screened_.weights().total_area().value*wr::TargetDepth;
  if(!std::isfinite(m.energy_)||m.energy_<=0||!std::isfinite(m.momentum_)||m.momentum_<=0||
     !std::isfinite(m.force_scale_)||m.force_scale_<=0) { error="Invalid wall response physical scales"; return false; }
  State state; Results results; std::array<double,18> zero{};
  free_response::detail::Visit(f,state,results,zero,zero,zero,
    [&](const char* group,unsigned entity,unsigned component,double,const char* unit,double,bool compare) {
      const unsigned offset=static_cast<unsigned>(m.dictionary_.size()); double scale=1;
      const bool node=std::strncmp(group,"node.",5)==0;
      auto same=[&](const char* s) { return std::strcmp(group,s)==0; };
      if(node) {
        auto& n=m.node_fields_[entity];
        if(same("node.position")) { scale=wr::TargetDepth; if(!component)n.x=offset; }
        else if(same("node.quaternion_wxyz")) { if(!component)n.q=offset; }
        else if(same("node.carried_velocity")) { scale=wr::ImpactSpeed; if(!component)n.v=offset; }
        else if(same("node.carried_omega")) { scale=wr::ImpactSpeed/free_response::Side; if(!component)n.w=offset; }
        else if(same("node.synchronous_velocity")) { scale=wr::ImpactSpeed; if(!component)n.vs=offset; }
        else if(same("node.synchronous_omega")) { scale=wr::ImpactSpeed/free_response::Side; if(!component)n.ws=offset; }
        else if(same("node.rotation_vector")) { if(!component)n.angle=offset; }
      } else {
        auto& e=m.element_fields_[entity];
        if(same("element.total_stress")) { scale=free_response::Young; if(!component)e.stress=offset; }
        else if(same("element.material_stress")) { scale=free_response::Young; if(!component)e.material=offset; }
        else if(same("element.bending_stress")) { scale=free_response::Young; if(!component)e.bending=offset; }
        else if(same("element.hourglass")) { scale=free_response::Young/((component==2||component==3||component==8||component==9)?free_response::Side:1.); if(!component)e.hourglass=offset; }
        else if(same("element.strain_curvature")) { scale=component<5?1:1/free_response::Side; if(!component)e.strain=offset; }
        else if(same("element.reported_thickness")) { scale=free_response::Thickness; e.thickness=offset; }
        else if(same("element.internal_work")) { scale=m.energy_; if(!component)e.work=offset; }
        else if(same("element.viscous_work")) { scale=m.energy_; e.viscous=offset; }
        else if(same("element.active")) e.active=offset;
        else if(same("element.cached_force")) { scale=m.force_scale_; if(!component)e.force=offset; }
        else if(same("element.cached_couple")) { scale=m.force_scale_*free_response::Side; if(!component)e.couple=offset; }
        else if(same("element.geometry")&&!component) e.geometry=offset;
      }
      m.dictionary_.push_back({std::string(group)+"["+std::to_string(entity)+"]["+std::to_string(component)+"]",unit,scale,compare});
    });
  m.native_field_count_=static_cast<unsigned>(m.dictionary_.size());
  for(unsigned n=0;n<f.nodes;++n) m.dictionary_.push_back({"contact.node_force["+std::to_string(n)+"]","N",m.force_scale_,true});
  m.dictionary_.push_back({"contact.resultant","N",m.force_scale_,true});
  m.dictionary_.push_back({"contact.potential","J",m.energy_,true});
  m.dictionary_.push_back({"wall.kick_impulse","N*s",2*m.momentum_,false});
  m.dictionary_.push_back({"wall.synchronous_impulse","N*s",2*m.momentum_,true});
  if(m.dictionary_.size()>MaxFields) { error="Wall response field capacity exceeded"; return false; }
  m.prepared_=true; output=std::move(m); error.clear(); return true;
}
} // namespace tl::qualification::qeph::wall_response
