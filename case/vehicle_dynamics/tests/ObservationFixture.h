#pragma once
#include "../MotionSummary.h"
#include "lib_src/solvers/NodalUniformMotionObserver.h"
#include "lib_src/solvers/ExplicitNodalStep.h"
#include "lib_src/solvers/NodalTrialIdentity.h"
#include <cstring>
#include <stdexcept>
namespace crash::cases::vehicle_dynamics::motion_test {
namespace fe=tl::fea;
inline void Require(fe::NodalReport r) {
  if(r.status!=fe::NodalStatus::Ok)throw std::runtime_error(r.message);
}
// Generic all-node physical-owner observation fixture, no Yaris/source shortcut.
// It is a performance fixture, not a material/contact trajectory benchmark.
struct Fixture {
  fe::FENodalState owner;
  fe::NodalUniformMotionObserver observer;
  fe::NodalNodeDomain domain;
  Fields fields;
  fe::NodalTrialToken token;
  fe::NodalPreparedView prepared;
  std::size_t nodes;
  explicit Fixture(std::size_t n):fields(n),nodes(n) {
    if(!n||n>fe::MaxActiveNodalStateNodes)throw std::runtime_error("Unsupported observation size");
    std::vector<fe::NodalDomainNode> source(n);
    std::vector<double> inverse(n,1);
    std::vector<std::uint8_t> fixed(n);
    for(std::size_t i=0;i<n;++i) {
      const tl::math::Vec3 x{double(i%127)*.125,double(i%17)*-.0625,double(i%31)*.03125};
      source[i]={i+1,x};
      fields.position[3*i]=x.x;fields.position[3*i+1]=x.y;fields.position[3*i+2]=x.z;
      fields.velocity[3*i]=16+.125*double(i%7);
      fields.velocity[3*i+1]=double(i%5)*-.0625;
      fields.spin[3*i+2]=.015625*double(i%3);
      fields.orientation[4*i]=1;
    }
    const auto bound=domain.Initialize({871,source.data(),n},fe::NodalDomainLimits::Vehicle());
    if(!bound)throw std::runtime_error(bound.message);
    fe::NodalStateConfig config;
    config.node_count=n;config.max_nodes=fe::MaxActiveNodalStateNodes;
    config.max_device_bytes=fe::MaxActiveNodalStateDeviceBytes;
    config.fixed_dt=1e-6;config.temporal_scheme=fe::NodalTemporalScheme::StaggeredHalfKickStart;
    Require(owner.Initialize(config,{fields.position.data(),fields.velocity.data(),fields.spin.data(),n,
        fields.orientation.data()},inverse.data(),fe::NodalDofConfig{fixed.data(),fixed.data(),inverse.data()}));
    // Source-domain equality is checked independently before the observer
    // captures this actual owner. No copied observer reference is used here.
    fe::NodalStamp initial_stamp;
    Require(owner.CopyAccepted(fields.buffer(),&initial_stamp));
    if(initial_stamp.epoch!=0)throw std::runtime_error("Initial observation source is not fresh");
    for(std::size_t i=0;i<n;++i) {
      const auto x=domain.nodes()[i].position;
      const double expected[]{x.x,x.y,x.z};
      if(std::memcmp(fields.position.data()+3*i,expected,sizeof(expected))!=0)
        throw std::runtime_error("Owner initial coordinate bits differ from source domain");
    }
    Require(observer.Initialize(owner));
    fe::NodalAssemblyView assembly;
    Require(owner.BeginTrial(&token,&assembly));
    Require(owner.SealAssembly(token));
    Require(fe::AdvanceStaggeredPrescribed(owner,token,
        {assembly.owner_id,assembly.accepted.base_epoch,assembly.attempt,config.fixed_dt,.2}));
    Require(owner.BorrowPrepared(token,&prepared));
  }
  ~Fixture(){owner.Discard();}
  MotionSummary Old() {
    fe::NodalPreparedView actual;
    Require(owner.CopyPrepared(token,fields.buffer(),&actual));
    if(!fe::trial_identity::SamePrepared(prepared,actual))throw std::runtime_error("Old observation phase differs");
    return ObserveUniformMotion(domain,fields,16,actual.proposed_time);
  }
  MotionSummary New() {
    fe::NodalUniformMotionObservation out;
    Require(observer.ObservePrepared(owner,token,{16,0,0},&out));
    if(!fe::trial_identity::SamePrepared(prepared,out.prepared))throw std::runtime_error("New observation phase differs");
    return {out.motion.nodes,out.motion.maximum_position_error,out.motion.maximum_velocity_error,
        out.motion.maximum_orientation_error,out.motion.maximum_spin};
  }
};
inline bool Same(const MotionSummary& a,const MotionSummary& b) {
  const double x[]{a.maximum_position_error,a.maximum_velocity_error,a.maximum_orientation_error,a.maximum_spin};
  const double y[]{b.maximum_position_error,b.maximum_velocity_error,b.maximum_orientation_error,b.maximum_spin};
  return a.nodes==b.nodes&&std::memcmp(x,y,sizeof(x))==0;
}
}
