#include "RecurrenceAudit.h"
#include <cmath>

namespace tl::qualification::qeph::recurrence {
namespace {
void Append(Model& m,Group group,unsigned entity,unsigned component,double scale,bool feedback,
            const std::string& name,const char* unit) {
  m.dictionary.push_back({group,entity,component,scale,feedback,name,unit});
}
void Dictionary(Model& m) {
  const double speed=std::sqrt(Young/Density);
  const char* groups[]{"position","orientation_tangent","velocity","spin"};
  const char* units[]{"m","rad","m/s","rad/s"};
  const double scales[]{Length,1.,speed,speed/Length};
  for(unsigned g=0;g<4;++g) for(unsigned n=0;n<m.nodes;++n) for(unsigned a=0;a<3;++a)
    Append(m,static_cast<Group>(g),n,a,scales[g],g>=2,std::string(groups[g])+"_"+std::to_string(n)+"_"+std::to_string(a),units[g]);
  for(unsigned e=0;e<m.elements;++e) {
    for(unsigned c=0;c<37;++c) {
      double scale=Young; const char* unit="Pa"; std::string field;
      if(c<5) field="total_stress_"+std::to_string(c);
      else if(c<10) field="material_stress_"+std::to_string(c-5);
      else if(c<13) { field="bending_stress_"+std::to_string(c-10); scale=Young*Thickness/Length; }
      else if(c<25) { const auto i=c-13; field="stabilization_"+std::to_string(i);
        if(i==2||i==3||i==8||i==9) { scale=Young/Length; unit="Pa/m"; } }
      else if(c<33) { field="strain_curvature_"+std::to_string(c-25); scale=c<30?1.:1./Length; unit=c<30?"1":"1/m"; }
      else if(c==33) { field="reported_thickness"; scale=Thickness; unit="m"; }
      else { field=c==36?"hourglass_viscous_work":"internal_work_"+std::to_string(c-34);
        scale=c==35?Young*Thickness*Thickness*Thickness:Young*Thickness*Length*Length; unit="J"; }
      Append(m,Group::History,e,c,scale,c>=5&&c<25,"element_"+std::to_string(e)+"_"+field,unit);
    }
    for(unsigned c=0;c<24;++c) Append(m,Group::ForceCache,e,c,
      Young*Thickness*Length*(c<12?1.:Length),true,
      "element_"+std::to_string(e)+(c<12?"_force_":"_couple_")+std::to_string(c%12),c<12?"N":"N*m");
  }
}
}
bool BuildModel(unsigned cells,Model& output,std::string& error) {
  if(cells<1||cells>2) { error="Only the frozen one/two-cell fixtures are admitted"; return false; }
  Model m; m.elements=cells; m.nodes=2*(cells+1);
  for(unsigned n=0;n<m.nodes;++n) m.position[n]={Length*(n/2),Length*(n%2?.5:-.5),0};
  for(unsigned e=0;e<cells;++e) {
    m.connectivity[e]={{2*e,2*e+2,2*e+3,2*e+1}};
    ReferenceInput in; in.density=Density; in.young_modulus=Young; in.poisson_ratio=Poisson; in.thickness=Thickness;
    for(unsigned i=0;i<4;++i) { const auto n=m.connectivity[e][i]; in.position[i]=m.position[n]; in.node_ids[i]=100+n; }
    if(Initialize(in,m.reference[e])!=Status::kSuccess) { error="Native frozen fixture startup failed"; return false; }
    for(unsigned i=0;i<4;++i) { const auto n=m.connectivity[e][i];
      m.mass[n]+=m.reference[e].data().nodal_mass[i]; m.inertia[n]+=m.reference[e].data().isotropic_inertia[i]; }
  }
  Dictionary(m); output=std::move(m); error.clear(); return true;
}
std::vector<unsigned> FeedbackIndices(const Model& m) {
  std::vector<unsigned> indices;
  for(unsigned i=0;i<m.dictionary.size();++i) if(m.dictionary[i].feedback) indices.push_back(i);
  return indices;
}
} // namespace tl::qualification::qeph::recurrence
