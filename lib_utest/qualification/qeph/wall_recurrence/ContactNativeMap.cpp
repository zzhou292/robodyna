#include "ContactBranchProbe.h"
#include "lib_utest/qualification/qeph/free_response/RecurrenceNativeMap.h"
#include <cmath>
#include <utility>

namespace tl::qualification::qeph::wall_recurrence {
namespace r=recurrence;
bool EvaluateContactNativeMap(const WallRecurrenceModel& model,double h,double normal_velocity,
                              const Eigen::VectorXd& input,ContactMapSample& output,std::string& error) {
  if(!model.prepared()||!FrozenStep(h)||!FrozenVelocity(normal_velocity)||
     input.size()!=static_cast<Eigen::Index>(model.native().dictionary.size())||!input.allFinite()) {
    error="Malformed or unfrozen contact/native operands"; return false;
  }
  const auto& m=model.native();
  std::array<double,3*r::MaxNodes> position{},velocity{};
  std::array<double,r::MaxNodes> inverse{};
  std::array<std::uint8_t,r::MaxNodes> fixed{};
  unsigned active=0;
  for(unsigned n=0;n<m.nodes;++n) {
    const double base_position[]{m.position[n].x,m.position[n].y,m.position[n].z};
    for(unsigned a=0;a<3;++a) {
      const auto x=model.coordinate(r::Group::Position,n,a),v=model.coordinate(r::Group::Velocity,n,a);
      position[3*n+a]=base_position[a]+input[x]*m.dictionary[x].scale;
      velocity[3*n+a]=(a==0?normal_velocity:0)+input[v]*m.dictionary[v].scale;
      if(!std::isfinite(position[3*n+a])||!std::isfinite(velocity[3*n+a])) {
        error="Nonfinite represented contact positions/velocities"; return false;
      }
    }
    if(std::abs(position[3*n+1])>MotionExtent||std::abs(position[3*n+2])>MotionExtent) {
      error="Probe leaves the certified finite-wall motion box"; return false;
    }
    if(position[3*n]>=model.law().wall_x) ++active;
    inverse[n]=1/m.mass[n];
  }
  if(active!=0&&active!=m.nodes) { error="Mixed contact mask is outside coherent broadside probes"; return false; }
  contact::NodalWallResult physical;
  const auto report=contact::EvaluateNodalWallContact(model.weights(),{position.data(),m.nodes,3,1},
    {velocity.data(),m.nodes,3,1},{inverse.data(),fixed.data(),m.nodes,1,contact::TranslationMassModel::kIsotropicLumped},
    model.law(),1,&physical);
  if(report.status!=contact::NodalWallStatus::Ok) {
    error="Owning host contact rejected: status="+std::to_string(static_cast<int>(report.status))+
          " parent="+std::to_string(report.parent)+" node="+std::to_string(report.node); return false;
  }
  Eigen::VectorXd kicked=input;
  for(unsigned i=0;i<physical.node_count;++i) {
    const auto& node=physical.nodes[i]; const auto v=model.coordinate(r::Group::Velocity,node.node,0);
    const long double increment=static_cast<long double>(h)*node.force_world.x/m.mass[node.node]/m.dictionary[v].scale;
    kicked[v]=static_cast<double>(static_cast<long double>(input[v])+increment);
  }
  if(!kicked.allFinite()) { error="Nonfinite normalized physical contact kick"; return false; }
  ContactMapSample result;
  if(!r::NativeMapWithUniformVelocity(m,h,{normal_velocity,0,0},kicked,result.state,error)) return false;
  result.nodes.assign(physical.nodes.begin(),physical.nodes.begin()+physical.node_count);
  result.parents.assign(physical.parents.begin(),physical.parents.begin()+physical.parent_count);
  result.resultant=physical.resultant; result.potential=physical.potential;
  result.wall_reaction=physical.wall_reaction; result.wall_moment=physical.wall_moment;
  result.surface_power=physical.surface_power; result.base_epoch=physical.base_epoch; result.attempt=physical.attempt;
  output=std::move(result); error.clear(); return true;
}
} // namespace tl::qualification::qeph::wall_recurrence
