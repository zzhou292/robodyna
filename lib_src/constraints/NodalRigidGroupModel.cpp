// SPDX-License-Identifier: AGPL-3.0-or-later
#include "NodalRigidGroupModel.h"
#include <Eigen/Eigenvalues>
#include <algorithm>
#include <cmath>
#include <limits>
#include <new>
#include <numeric>
#include <stdexcept>
#include <vector>

namespace tl::fea {
struct NodalRigidGroupModel::Impl {
  std::uint64_t source_instance=0;
  NodalRigidSourceUnits units{};
  std::size_t node_count=0,owned_bytes=0,startup_bytes=0;
  std::vector<NodalRigidGroupProperties> groups;
  std::vector<NodalRigidGroupMember> members;
};
namespace {
using Status=NodalRigidGroupStatus;
using Report=NodalRigidGroupReport;
using Vec3=tl::math::Vec3;
Report Fail(Status code,const char* message,std::size_t group=SIZE_MAX,std::size_t member=SIZE_MAX) {
  return {code,message,group,member};
}
bool Positive(double v) { return std::isfinite(v)&&v>0; }
bool AddBytes(std::size_t count,std::size_t width,std::size_t& bytes) {
  if(count>(SIZE_MAX-bytes)/width) return false;
  bytes+=count*width; return true;
}
Eigen::Vector3d EigenVector(Vec3 v) { return {v.x,v.y,v.z}; }
Vec3 Value(const Eigen::Vector3d& v) { return {v[0],v[1],v[2]}; }
tl::math::Matrix3 Value(const Eigen::Matrix3d& m) {
  tl::math::Matrix3 out;
  for(unsigned i=0;i<3;++i) for(unsigned j=0;j<3;++j) out.v[3*i+j]=m(i,j);
  return out;
}
bool ValidMass(const NodalRigidGroupMember& m) {
  if(!Positive(m.mass_kg)||!Positive(m.total_inertia_kg_m2)||
     !Positive(m.physical_inertia_kg_m2)||!std::isfinite(m.added_inertia_kg_m2)||m.added_inertia_kg_m2<0)
    return false;
  const double partitions=m.physical_inertia_kg_m2+m.added_inertia_kg_m2;
  // Native total J and its partitions can have different final roundoff.
  // This check does not replace the authoritative total with their sum.
  return std::isfinite(partitions)&&
    std::fabs(partitions-m.total_inertia_kg_m2)<=
      64*std::numeric_limits<double>::epsilon()*std::max(partitions,m.total_inertia_kg_m2);
}
bool AddInertia(Eigen::Matrix3d& tensor,double mass,double inertia,Eigen::Vector3d r) {
  const double xx=r[0]*r[0],yy=r[1]*r[1],zz=r[2]*r[2];
  const double xy=r[0]*r[1],xz=r[0]*r[2],yz=r[1]*r[2];
  tensor(0,0)+=inertia+(yy+zz)*mass;
  tensor(1,1)+=inertia+(zz+xx)*mass;
  tensor(2,2)+=inertia+(xx+yy)*mass;
  tensor(0,1)-=xy*mass; tensor(1,0)-=xy*mass;
  tensor(0,2)-=xz*mass; tensor(2,0)-=xz*mass;
  tensor(1,2)-=yz*mass; tensor(2,1)-=yz*mass;
  return r.allFinite()&&tensor.allFinite();
}
Report PrepareGroup(NodalRigidGroupProperties& g,const NodalRigidGroupMember* members,
    double primary_mass,double primary_j,std::size_t group) {
  Eigen::Vector3d geometric=Eigen::Vector3d::Zero(),moment=Eigen::Vector3d::Zero();
  for(std::size_t i=0;i<g.member_count;++i) {
    const auto& m=members[i];
    geometric+=EigenVector(m.position); moment+=m.mass_kg*EigenVector(m.position);
    g.structural_mass_kg+=m.mass_kg;
    g.native_total_inertia_sum+=m.total_inertia_kg_m2;
    g.physical_inertia_sum+=m.physical_inertia_kg_m2; g.added_inertia_sum+=m.added_inertia_kg_m2;
    if(!geometric.allFinite()||!moment.allFinite()||!Positive(g.structural_mass_kg)||
       !Positive(g.native_total_inertia_sum)||!Positive(g.physical_inertia_sum)||!std::isfinite(g.added_inertia_sum))
      return Fail(Status::NonfiniteResult,"Rigid group mass/centroid accumulation overflow",group,i);
  }
  geometric/=static_cast<double>(g.member_count);
  g.generated_primary_position=Value(geometric);
  g.structural_center=Value(Eigen::Vector3d(moment/g.structural_mass_kg));
  // Match the source's declared primary-first mass/COG accumulation order.
  double mass=primary_mass; Eigen::Vector3d weighted=geometric*primary_mass;
  for(std::size_t i=0;i<g.member_count;++i) {
    mass+=members[i].mass_kg; weighted+=members[i].mass_kg*EigenVector(members[i].position);
  }
  if(!Positive(mass)||!weighted.allFinite()) return Fail(Status::NonfiniteResult,"Nonfinite effective group mass",group);
  const Eigen::Vector3d center=weighted/mass;
  if(!center.allFinite()) return Fail(Status::NonfiniteResult,"Nonfinite rigid center of mass",group);
  g.center=Value(center); g.total_mass_kg=mass;
  g.regularization.primary_mass_kg=primary_mass;
  g.regularization.primary_isotropic_inertia_kg_m2=primary_j;
  Eigen::Matrix3d tensor=Eigen::Matrix3d::Zero();
  if(!AddInertia(tensor,primary_mass,primary_j,geometric-center))
    return Fail(Status::NonfiniteResult,"Nonfinite primary inertia",group);
  for(std::size_t i=0;i<g.member_count;++i)
    if(!AddInertia(tensor,members[i].mass_kg,members[i].total_inertia_kg_m2,EigenVector(members[i].position)-center))
      return Fail(Status::NonfiniteResult,"Rigid member inertia overflow",group,i);
  const double scale=tensor.cwiseAbs().maxCoeff();
  if(!Positive(scale)) return Fail(Status::EigenFailure,"Invalid inertia tensor scale",group);
  // Fixed-size Eigen storage; axes/sign/order are an equivalent tensor
  // factorization, not a claim of native VALPR bitwise eigenvector identity.
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen(tensor/scale);
  if(eigen.info()!=Eigen::Success) return Fail(Status::EigenFailure,"Principal inertia decomposition failed",group);
  Eigen::Vector3d raw=eigen.eigenvalues()*scale;
  Eigen::Matrix3d axes=eigen.eigenvectors();
  if(axes.determinant()<0) axes.col(2)*=-1;
  rigid::PrincipalCorrection correction;
  if(rigid::CorrectPrincipalInertia(Value(raw),correction)!=rigid::MathStatus::Success)
    return Fail(Status::EigenFailure,"Principal inertia cannot be admitted after source correction",group);
  g.principal={Value(axes),correction.effective};
  if(!rigid::detail::Orthonormal(g.principal.axes))
    return Fail(Status::EigenFailure,"Principal frame is not a proper orthonormal frame",group);
  const Eigen::Matrix3d added=axes*EigenVector(correction.added).asDiagonal()*axes.transpose();
  const Eigen::Matrix3d effective=tensor+added;
  if(!added.allFinite()||!effective.allFinite()) return Fail(Status::NonfiniteResult,"Effective group tensor overflow",group);
  g.raw_tensor=Value(tensor); g.effective_tensor=Value(effective); g.raw_principal_inertia=Value(raw);
  g.regularization.principal_inertia_added=correction.added;
  g.regularization.tensor_added=Value(added);
  g.regularization.principal_threshold_reached=correction.threshold_reached;
  g.regularization.principal_inertia_changed=correction.changed;
  return {};
}
} // namespace

NodalRigidGroupModel::NodalRigidGroupModel()=default;
NodalRigidGroupModel::~NodalRigidGroupModel()=default;
NodalRigidGroupReport NodalRigidGroupModel::Initialize(const NodalRigidGroupModelInput& input) noexcept {
  if(impl_) return Fail(Status::AlreadyInitialized,"Rigid group model is immutable after initialization");
  const auto& l=input.limits;
  if(!input.source_instance_id||!input.global_node_count||!input.groups||!input.group_count||
     !l.max_groups||!l.max_members||l.max_members_per_group<3||!l.max_host_bytes||
     !Positive(input.source_units.mass_to_kg)||!Positive(input.source_units.length_to_m))
    return Fail(Status::InvalidInput,"Rigid model requires explicit identity, complete groups, bounds and source units");
  if(input.group_count>l.max_groups) return Fail(Status::ResourceLimit,"Rigid group count exceeds admission cap");
  std::size_t forecast=sizeof(Impl);
  if(input.group_count>l.max_members/3||!AddBytes(input.group_count,sizeof(NodalRigidGroupProperties),forecast)||
     forecast>l.max_host_bytes) return Fail(Status::ResourceLimit,"Rigid group inventory cannot fit startup budget");
  std::size_t count=0;
  for(std::size_t g=0;g<input.group_count;++g) {
    const auto& in=input.groups[g];
    if(!in.source_group_id||!in.source_node_set_id||!in.members||in.member_count<3)
      return Fail(Status::InvalidInput,"Rigid group requires explicit source IDs and at least three complete members",g);
    if(in.member_count>l.max_members_per_group||count>l.max_members||in.member_count>l.max_members-count)
      return Fail(Status::ResourceLimit,"Rigid membership count exceeds admission cap",g);
    count+=in.member_count;
  }
  if(!AddBytes(count,sizeof(NodalRigidGroupMember),forecast)||!AddBytes(count,sizeof(std::size_t),forecast)||
     forecast>l.max_host_bytes) return Fail(Status::ResourceLimit,"Rigid startup payload exceeds byte budget");
  const double primary_mass=1e-20*input.source_units.mass_to_kg;
  const double primary_j=primary_mass*input.source_units.length_to_m*input.source_units.length_to_m;
  if(!Positive(primary_mass)||!Positive(primary_j))
    return Fail(Status::InvalidInput,"Source regularizers are not positive finite SI values");
  try {
    auto next=std::make_unique<Impl>();
    next->groups.resize(input.group_count); next->members.reserve(count);
    std::vector<std::size_t> order(count);
    for(std::size_t g=0;g<input.group_count;++g) {
      const auto& in=input.groups[g]; auto& out=next->groups[g];
      out.source_group_id=in.source_group_id; out.source_node_set_id=in.source_node_set_id;
      out.member_offset=next->members.size(); out.member_count=in.member_count;
      for(std::size_t i=0;i<in.member_count;++i) {
        const auto& m=in.members[i];
        if(!m.source_node_id||m.global_node>=input.global_node_count||!rigid::detail::Finite(m.position))
          return Fail(Status::InvalidInput,"Rigid member identity/index/position is invalid",g,i);
        if(!ValidMass(m)) return Fail(Status::InvalidMass,"Rigid member native mass/J or partition evidence is invalid",g,i);
        next->members.push_back(m);
      }
    }
    std::iota(order.begin(),order.begin()+input.group_count,0);
    for(unsigned pass=0;pass<2;++pass) {
      const auto key=[&](std::size_t i) {
        return pass==0?next->groups[i].source_group_id:next->groups[i].source_node_set_id;
      };
      std::sort(order.begin(),order.begin()+input.group_count,[&](auto a,auto b) {
        return key(a)<key(b)||(key(a)==key(b)&&a<b);
      });
      for(std::size_t i=1;i<input.group_count;++i)
        if(key(order[i-1])==key(order[i]))
          return Fail(Status::DuplicateIdentity,
            pass==0?"Repeated source rigid-group ID":"Repeated source node-set ID",order[i]);
    }
    std::iota(order.begin(),order.end(),0);
    for(unsigned pass=0;pass<2;++pass) {
      const auto key=[&](std::size_t i)->std::uint64_t { return pass==0?next->members[i].source_node_id:next->members[i].global_node; };
      std::sort(order.begin(),order.end(),[&](auto a,auto b) { return key(a)<key(b)||(key(a)==key(b)&&a<b); });
      for(std::size_t i=1;i<count;++i) if(key(order[i-1])==key(order[i])) {
        std::size_t group=0;
        while(group+1<input.group_count&&next->groups[group+1].member_offset<=order[i]) ++group;
        return Fail(pass==0?Status::DuplicateIdentity:Status::DuplicateMembership,
          pass==0?"Source node occurs more than once across rigid members":"Global node occurs more than once across rigid members",
          group,order[i]-next->groups[group].member_offset);
      }
    }
    next->owned_bytes=sizeof(Impl);
    if(!AddBytes(next->groups.capacity(),sizeof(NodalRigidGroupProperties),next->owned_bytes)||
       !AddBytes(next->members.capacity(),sizeof(NodalRigidGroupMember),next->owned_bytes))
      return Fail(Status::ResourceLimit,"Rigid storage capacity overflow");
    next->startup_bytes=next->owned_bytes;
    if(!AddBytes(order.capacity(),sizeof(std::size_t),next->startup_bytes)||next->startup_bytes>l.max_host_bytes)
      return Fail(Status::ResourceLimit,"Actual rigid startup capacity exceeds payload budget");
    for(std::size_t g=0;g<input.group_count;++g) {
      auto report=PrepareGroup(next->groups[g],next->members.data()+next->groups[g].member_offset,primary_mass,primary_j,g);
      if(!report) return report;
    }
    next->source_instance=input.source_instance_id; next->units=input.source_units; next->node_count=input.global_node_count;
    impl_=std::move(next); return {};
  } catch(const std::bad_alloc&) { return Fail(Status::ResourceLimit,"Rigid startup allocation failed"); }
    catch(const std::length_error&) { return Fail(Status::ResourceLimit,"Rigid startup size is not representable"); }
}
bool NodalRigidGroupModel::prepared() const noexcept { return bool(impl_); }
std::uint64_t NodalRigidGroupModel::source_instance_id() const noexcept { return impl_?impl_->source_instance:0; }
NodalRigidSourceUnits NodalRigidGroupModel::source_units() const noexcept { return impl_?impl_->units:NodalRigidSourceUnits{}; }
std::size_t NodalRigidGroupModel::global_node_count() const noexcept { return impl_?impl_->node_count:0; }
std::size_t NodalRigidGroupModel::group_count() const noexcept { return impl_?impl_->groups.size():0; }
std::size_t NodalRigidGroupModel::member_count() const noexcept { return impl_?impl_->members.size():0; }
std::size_t NodalRigidGroupModel::owned_payload_bytes() const noexcept { return impl_?impl_->owned_bytes:0; }
std::size_t NodalRigidGroupModel::startup_payload_bytes() const noexcept { return impl_?impl_->startup_bytes:0; }
const NodalRigidGroupProperties* NodalRigidGroupModel::groups() const noexcept { return impl_?impl_->groups.data():nullptr; }
const NodalRigidGroupMember* NodalRigidGroupModel::members() const noexcept { return impl_?impl_->members.data():nullptr; }
} // namespace tl::fea
