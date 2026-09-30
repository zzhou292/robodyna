#include "ResponseData.h"
#include "ResponseFieldVisitor.h"
#include "lib_src/elements/qeph/QephStartup.h"
#include <cmath>

namespace tl::qualification::qeph::response {
bool ValidConfig(Config c) noexcept { return (c.cells==1||c.cells==2)&&(c.refinement==1||c.refinement==2||c.refinement==4); }
double BendingScale() noexcept { return Young*Thickness*Thickness*Thickness/(12*(1-Poisson*Poisson)); }
double PulseFactor(double t) noexcept {
  if(!std::isfinite(t)||t<0) return NAN;
  if(t==0||t>=Pulse) return 0; // Exact endpoints of the declared compact pulse.
  const double s=std::sin(std::acos(-1.)*t/Pulse); return s*s;
}
bool BuildModel(unsigned cells,Model& output,std::string& error) {
  if(cells!=1&&cells!=2) { error="Only the frozen one/two-cell models are supported"; return false; }
  Model m; m.cells=cells; m.nodes=2*(cells+1);
  for(unsigned n=0;n<m.nodes;++n) { m.initial_position[3*n]=Side*(n/2); m.initial_position[3*n+1]=Side*(n%2?.5:-.5); }
  for(unsigned e=0;e<cells;++e) {
    m.connectivity[e]={2*e,2*e+2,2*e+3,2*e+1}; port::ReferenceInput in;
    in.young_modulus=Young; in.density=Density; in.thickness=Thickness; in.poisson_ratio=Poisson;
    for(unsigned i=0;i<4;++i) { const auto n=m.connectivity[e][i]; in.node_ids[i]=100+n;
      in.position[i]={m.initial_position[3*n],m.initial_position[3*n+1],m.initial_position[3*n+2]}; }
    if(port::InitializeReference(in,m.reference[e])!=port::Status::kSuccess) { error="Frozen reference rejected"; return false; }
    for(unsigned i=0;i<4;++i) { const auto n=m.connectivity[e][i];
      m.mass[n]+=m.reference[e].nodal_mass[i]; m.inertia[n]+=m.reference[e].isotropic_inertia[i];
      m.physical[n]+=m.reference[e].physical_inertia[i]; m.added[n]+=m.reference[e].added_inertia[i]; }
  }
  output=m; error.clear(); return true;
}
double ExperimentEnergy(const Model& m) noexcept {
  // Sum of absolute frozen load amplitudes times their displacement scales.
  return m.cells==1?2*BendingScale()*Theta*Theta:4*BendingScale()*Delta*Delta/(Side*Side);
}
std::vector<Field> Dictionary(const Model& m) {
  std::vector<Field> fields; fields.reserve(MaxFields); State state; Results result; std::array<double,18> zero{};
  detail::Visit(m,state,result,zero,zero,zero,[&](const char* group,unsigned e,unsigned c,double,const char* unit,double scale,bool compare) {
    fields.push_back({std::string(group)+"["+std::to_string(e)+"]["+std::to_string(c)+"]",unit,scale,compare});
  });
  return fields;
}
} // namespace tl::qualification::qeph::response
