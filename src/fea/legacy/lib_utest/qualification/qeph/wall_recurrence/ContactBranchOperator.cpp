#include "ContactBranchProbe.h"
#include <cmath>
#include <utility>

namespace tl::qualification::qeph::wall_recurrence {
namespace r=recurrence;
namespace {
bool Valid(ContactBranch b) { return b==ContactBranch::Inactive||b==ContactBranch::Active; }
}
bool BuildContactKick(const WallRecurrenceModel& model,double h,ContactBranch branch,
                      Eigen::MatrixXd& output,std::string& error) {
  if(!model.prepared()||!FrozenStep(h)||!Valid(branch)) {
    error="Unprepared model or unfrozen contact branch/step"; return false;
  }
  const auto& m=model.native();
  Eigen::MatrixXd result=Eigen::MatrixXd::Identity(m.dictionary.size(),m.dictionary.size());
  if(branch==ContactBranch::Active) for(unsigned i=0;i<model.touching().node_count;++i) {
    const auto& node=model.touching().nodes[i]; const auto n=node.node;
    const auto x=model.coordinate(r::Group::Position,n,0),v=model.coordinate(r::Group::Velocity,n,0);
    const long double coefficient=static_cast<long double>(h)*node.stiffness.value/m.mass[n]*
                                  m.dictionary[x].scale/m.dictionary[v].scale;
    const double value=static_cast<double>(coefficient);
    if(!std::isfinite(value)||value<=0) { error="Nonfinite normalized contact-kick coefficient"; return false; }
    result(v,x)=-value;
  }
  output=std::move(result); error.clear(); return true;
}
bool BuildContactBranch(const WallRecurrenceModel& model,double h,ContactBranch branch,
                        const Eigen::MatrixXd& shell,Eigen::MatrixXd& output,std::string& error) {
  if(!model.prepared()||shell.rows()!=static_cast<Eigen::Index>(model.native().dictionary.size())||
     shell.rows()!=shell.cols()||!shell.allFinite()) {
    error="Incomplete full native shell operator"; return false;
  }
  Eigen::MatrixXd kick;
  if(!BuildContactKick(model,h,branch,kick,error)) return false;
  Eigen::MatrixXd result=(shell*kick).eval();
  if(!result.allFinite()) { error="Nonfinite full contact-branch composition"; return false; }
  output=std::move(result); error.clear(); return true;
}
bool SignConeDirections(const WallRecurrenceModel& model,ContactBranch branch,
                        std::vector<ContactDirection>& output,std::string& error) {
  if(!model.prepared()||!Valid(branch)) { error="Unprepared sign-cone model/branch"; return false; }
  const auto& m=model.native(); const double sign=branch==ContactBranch::Active?1.:-1.;
  ContactDirection base{"uniform_normal_position",Eigen::VectorXd::Zero(m.dictionary.size())};
  for(unsigned n=0;n<m.nodes;++n) base.value[model.coordinate(r::Group::Position,n,0)]=sign*.5;
  std::vector<ContactDirection> result{base};
  for(unsigned n=0;n<m.nodes;++n) {
    auto d=base; d.name="normal_position_node_"+std::to_string(n);
    d.value[model.coordinate(r::Group::Position,n,0)]+=sign*.5; result.push_back(std::move(d));
  }
  const char* names[]{"uniform_normal_velocity","alternating_normal_velocity","alternating_spin_y",
                      "material_xx","stabilization_0","cached_world_x_force"};
  for(unsigned kind=0;kind<6;++kind) {
    auto d=base; d.name=names[kind];
    if(kind<3) for(unsigned n=0;n<m.nodes;++n) {
      const auto group=kind==2?r::Group::Spin:r::Group::Velocity;
      d.value[model.coordinate(group,n,kind==2?1:0)]=kind==0||n%2==0?1.:-1.;
    }
    else d.value[model.coordinate(kind==5?r::Group::ForceCache:r::Group::History,0,
                                  kind==3?5:(kind==4?13:0))]=1;
    result.push_back(std::move(d));
  }
  output=std::move(result); error.clear(); return true;
}
} // namespace tl::qualification::qeph::wall_recurrence
