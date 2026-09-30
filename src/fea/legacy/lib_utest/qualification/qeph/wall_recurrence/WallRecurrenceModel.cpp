#include "WallRecurrenceModel.h"
#include "lib_utest/q4_planar_geometry_fixture.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace tl::qualification::qeph::wall_recurrence {
namespace r=recurrence;
namespace b=contact::q4_bounds;
bool FrozenStep(double h) { return std::find(r::Steps.begin(),r::Steps.end(),h)!=r::Steps.end(); }
bool FrozenVelocity(double v) { return std::find(VelocityBaselines.begin(),VelocityBaselines.end(),v)!=VelocityBaselines.end(); }
unsigned WallRecurrenceModel::coordinate(r::Group group,unsigned entity,unsigned component) const {
  for(unsigned i=0;i<native_.dictionary.size();++i) {
    const auto& c=native_.dictionary[i];
    if(c.group==group&&c.entity==entity&&c.component==component) return i;
  }
  return static_cast<unsigned>(native_.dictionary.size());
}
bool BuildWallRecurrenceModel(unsigned cells,WallRecurrenceModel& output,std::string& error) {
  WallRecurrenceModel m;
  if(!r::BuildModel(cells,m.native_,error)) return false;
  auto& n=m.native_;
  // Preserve the existing complete dictionary and cyclic topology. Only this
  // named fixture's rigid pose/current touching baseline differs from BQ4.
  for(unsigned i=0;i<n.nodes;++i)
    n.position[i]={0,r::Length*(static_cast<double>(i/2)-.5*cells),r::Length*(i%2?.5:-.5)};
  n.mass.fill(0); n.inertia.fill(0);
  std::array<double,3*r::MaxNodes> reference_positions{},touching_positions{},velocity{},inverse{};
  std::array<std::uint8_t,r::MaxNodes> fixed{};
  std::array<contact::SurfaceQ4,r::MaxElements> parents{};
  for(unsigned e=0;e<cells;++e) {
    auto input=n.reference[e].data().input;
    for(unsigned i=0;i<4;++i) {
      const auto node=n.connectivity[e][i];
      input.position[i]=n.position[node]; input.position[i].x=-InitialGap;
      parents[e].nodes[i]=node;
    }
    if(Initialize(input,n.reference[e])!=Status::kSuccess) {
      error="Native broadside reference startup rejected"; return false;
    }
    parents[e].parent_element_id=1001+e; parents[e].feature_id=2001+e;
    for(unsigned i=0;i<4;++i) {
      const auto node=n.connectivity[e][i];
      n.mass[node]+=n.reference[e].data().nodal_mass[i];
      n.inertia[node]+=n.reference[e].data().isotropic_inertia[i];
    }
  }
  for(unsigned i=0;i<n.nodes;++i) {
    if(!std::isfinite(n.mass[i])||n.mass[i]<=0||!std::isfinite(n.inertia[i])||n.inertia[i]<=0) {
      error="Native broadside mass/inertia rejected"; return false;
    }
    inverse[i]=1/n.mass[i];
    if(!std::isfinite(inverse[i])||inverse[i]<=0) { error="Native broadside reciprocal mass rejected"; return false; }
    reference_positions[3*i]=-InitialGap;
    touching_positions[3*i]=0;
    reference_positions[3*i+1]=touching_positions[3*i+1]=n.position[i].y;
    reference_positions[3*i+2]=touching_positions[3*i+2]=n.position[i].z;
  }
  const auto prepared=m.reference_.Initialize({reference_positions.data(),n.nodes,3,1},parents.data(),cells);
  if(prepared.status!=contact::Q4ParametricStatus::Ok) { error="Owning Q4 contact reference rejected"; return false; }
  const contact::NodalWallParentInput inputs[]{{&m.reference_,0,nullptr},{&m.reference_,1,nullptr}};
  if(m.weights_.Initialize(n.nodes,inputs,cells).status!=contact::NodalWallStatus::Ok) {
    error="Owning nodal area weights rejected"; return false;
  }
  // Reuse the retained two-triangle finite-wall fixture; its IDs and source
  // faces remain available in wall(). This is not a canonical Yaris wall.
  const auto square=q4_planar_test::Square();
  if(m.wall_.Initialize(square.view()).status!=contact::PlanarContactStatus::Ok) {
    error="Named finite square wall rejected"; return false;
  }
  const contact::PlanarWallBox box{{0,-MotionExtent,-MotionExtent},{0,MotionExtent,MotionExtent}};
  if(contact::CheckPlanarWallBox(m.wall_,box,WallClearance,3001,contact::PlanarWallBoxMode::Exact,
                               &m.coverage_).status!=contact::PlanarContactStatus::Ok||!m.coverage_.covered) {
    error="Named broadside motion box is not covered by finite wall"; return false;
  }
  auto& chain=m.penalty_chain_; chain[0]={r::Density,r::Density};
  if(!b::Scale(chain[0],r::Thickness,&chain[1])||!b::Scale(chain[1],ImpactSpeed,&chain[2])||
     !b::Scale(chain[2],ImpactSpeed,&chain[3])||!b::DividePositive(chain[3],TargetDepth,&chain[4])||
     !b::DividePositive(chain[4],TargetDepth,&chain[5])) {
    error="Owning upward penalty calculation rejected"; return false;
  }
  m.law_={0,chain[5].upper,MaximumDepth,ParentForceError,ParentEnergyError};
  const auto result=contact::EvaluateNodalWallContact(m.weights_,{touching_positions.data(),n.nodes,3,1},
    {velocity.data(),n.nodes,3,1},{inverse.data(),fixed.data(),n.nodes,1,contact::TranslationMassModel::kIsotropicLumped},
    m.law_,1,&m.touching_);
  if(result.status!=contact::NodalWallStatus::Ok||!m.touching_.valid||m.touching_.node_count!=n.nodes) {
    error="Owning touching-law coefficient extraction rejected"; return false;
  }
  double minimum=std::numeric_limits<double>::infinity(),maximum=0;
  for(unsigned i=0;i<m.touching_.node_count;++i) {
    const auto& node=m.touching_.nodes[i]; const auto id=node.node;
    contact::Q4IntegralInterval ratio;
    if(id>=n.nodes||!contact::nodal_wall_detail::Certificate(node.stiffness,true)||
       !b::DividePositive({node.stiffness.lower,node.stiffness.upper},n.mass[id],&ratio)||
       !b::Certify(node.stiffness.value/n.mass[id],ratio,&m.mass_rates_[id])) {
      error="Owning stiffness/native mass ratio rejected"; return false;
    }
    const double value=m.mass_rates_[id].value;
    minimum=std::min(minimum,value); maximum=std::max(maximum,value);
    m.maximum_frequency_=std::max(m.maximum_frequency_,std::sqrt(value));
  }
  contact::Q4IntegralInterval difference,relative;
  if(!b::Difference(maximum,minimum,&difference)||!b::DividePositive(difference,minimum,&relative)||
     !std::isfinite(m.maximum_frequency_)||m.maximum_frequency_<=0) {
    error="Native mass-rate spread calculation rejected"; return false;
  }
  m.rate_spread_upper_=relative.upper;
  if(m.rate_spread_upper_>MassRateSpreadLimit) {
    error="Named coherent broadside mass-rate spread exceeds fixed budget"; return false;
  }
  m.prepared_=true; output=std::move(m); error.clear(); return true;
}
} // namespace tl::qualification::qeph::wall_recurrence
