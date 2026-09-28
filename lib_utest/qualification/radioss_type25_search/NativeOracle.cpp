#include "NativeMaps.h"
#include <array>
namespace type25_search_test {
extern "C" void rd_search_budget(const double*,const double*,const double*,const double*,const int*,double*,int*);
extern "C" void rd_search_extrema(const int*,const int*,const int*,const int*,const int*,const int*,const int*,
    const double*,const double*,const double*,double*,double*);
extern "C" void rd_search_gap(const int*,const double*,const double*,double*);
namespace {
std::array<double,24> Pack(const s::Extrema& e) {
  std::array<double,24> out{};
  const s::MotionExtrema* boxes[]{&e.secondary_displacement,&e.main_displacement,&e.secondary_velocity,&e.main_velocity};
  for(unsigned i=0;i<4;++i) {
    const auto& b=*boxes[i];
    out[6*i]=b.maximum.x;out[6*i+1]=b.maximum.y;out[6*i+2]=b.maximum.z;
    out[6*i+3]=b.minimum.x;out[6*i+4]=b.minimum.y;out[6*i+5]=b.minimum.z;
  }
  return out;
}
}
s::Budget NativeBudget(const s::Extrema& e,double margin,double dt,bool force) {
  const auto boxes=Pack(e);double out[6]{};int level=0,override=force;
  rd_search_budget(boxes.data(),&e.maximum_gap_change,&margin,&dt,&override,out,&level);
  s::Budget b;b.displacement=out[0];b.relative_speed=out[1];b.raw_motion=out[2];b.stored_motion=out[3];
  b.raw_distance=out[4];b.distance=out[5];b.velocity=static_cast<s::VelocityStatus>(level);
  b.requires_sort=out[4]<=0 || force;return b;
}
s::Extrema NativeExtrema(const s::Source& source,const s::Current& current,const s::Current& reference, std::vector<double>* normalized_stiffness) {
  const int nodes=source.physical_nodes,ns=source.secondaries,nm=source.mains,no=source.main_1d;
  std::vector<int> secondary(ns),main(nm),one_d(no);
  for(int i=0;i<ns;++i)secondary[i]=source.secondary_nodes[i]+1;
  for(int i=0;i<nm;++i)main[i]=NativeMain(source,current,source.main_nodes[i]);
  for(int i=0;i<no;++i)one_d[i]=NativeMain(source,current,source.main_1d_nodes[i]);
  const bool si=source.input_units==s::InputUnits::Si;
  const double length=si ? source.units.length_m : 1;
  const double velocity=si ? source.units.length_m/source.units.time_s : 1;
  std::vector<double> x(3*nodes),v(3*nodes);
  const auto saved=NativeReference(source,reference);
  for(int i=0;i<nodes;++i) {
    const auto a=current.positions.at(i),b=current.velocities.at(i);
    x[3*i]=a.x/length;x[3*i+1]=a.y/length;x[3*i+2]=a.z/length;
    v[3*i]=b.x/velocity;v[3*i+1]=b.y/velocity;v[3*i+2]=b.z/velocity;
  }
  // I25BUCE_CRIT observes pre-clamp activity, then mutates STFN for NSPMD=1.
  // Give the complete donor a private copy; never mutate borrowed fixture input.
  std::vector<double> stiffness;
  if (ns) stiffness.assign(current.secondary_stiffness, current.secondary_stiffness + ns);
  double boxes[24]{};
  rd_search_extrema(&nodes,&ns,&nm,&no,secondary.data(),main.data(),one_d.data(),x.data(),v.data(),saved.data(),stiffness.data(),boxes);
  if (normalized_stiffness) *normalized_stiffness = stiffness;
  s::Extrema e;s::MotionExtrema* target[]{&e.secondary_displacement,&e.main_displacement,&e.secondary_velocity,&e.main_velocity};
  for(unsigned i=0;i<4;++i) {
    target[i]->maximum={boxes[6*i],boxes[6*i+1],boxes[6*i+2]};
    target[i]->minimum={boxes[6*i+3],boxes[6*i+4],boxes[6*i+5]};
  }
  for(int i=0;i<ns;++i)if(current.secondary_stiffness[i]!=0)++e.secondary_uses;
  for(int i=0;i<nm;++i)if(main[i]>0)++e.main_uses;
  for(int i=0;i<no;++i)if(one_d[i]>0){++e.secondary_uses;++e.main_uses;}
  e.maximum_gap_change=0;
  if(source.gap_mode==s::GapMode::CurrentMainGaps) {
    const int n=source.main_segments;std::vector<double> now(n),old(n);
    for(int i=0;i<n;++i){now[i]=current.main_gaps[i]/length;old[i]=reference.main_gaps[i]/length;}
    rd_search_gap(&n,now.data(),old.data(),&e.maximum_gap_change);
  }
  return e;
}
} // namespace type25_search_test
