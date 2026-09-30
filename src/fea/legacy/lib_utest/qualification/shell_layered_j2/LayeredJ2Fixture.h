#pragma once
#include "lib_src/elements/qeph/QephLayeredJ2.h"
#include "lib_src/elements/qeph/QephStartup.h"
#include "lib_src/elements/t3/T3LayeredJ2.h"
#include "lib_src/elements/t3/T3Startup.h"
#include <stdexcept>

namespace layered_j2_test {
namespace sec=tl::fea::sections;
namespace q=tl::fea::qeph;
namespace t=tl::fea::t3;
inline constexpr double strains[]{0,.002,.01,.1,.4};
inline constexpr double yields[]{220e6,250e6,300e6,400e6,500e6};
inline sec::PointParameters Parameters() {
  sec::PointParameters p;
  if(tl::material::PrepareTabulatedShellPlasticity(200e9,.3,7890,{strains,yields,5},p)!=sec::PointStatus::Ok)
    throw std::runtime_error("Material preparation failed");
  return p;
}
inline sec::ShellLayeredJ2Input Input(const sec::PointParameters& p) {
  sec::ShellLayeredJ2Input in;
  in.reference_thickness=in.reported_thickness=.002;
  in.transverse_shear_modulus=p.shear_modulus*(5./6.);
  return in;
}
struct FamilyInputs {
  q::ReferenceData qr; t::ReferenceData tr;
  q::LayeredJ2History qh; t::LayeredJ2History th;
  q::PrescribedInterval qi; t::PrescribedInterval ti;
};
inline FamilyInputs Families(const sec::PointParameters& p) {
  FamilyInputs f;
  q::ReferenceInput qr; t::ReferenceInput tr;
  qr.thickness=tr.thickness=.002;
  qr.position[0]={0,0,0}; qr.position[1]={.02,0,0};
  qr.position[2]={.02,.02,0}; qr.position[3]={0,.02,0};
  tr.position[0]={0,0,0}; tr.position[1]={.02,0,0}; tr.position[2]={0,.02,0};
  if(q::InitializeReference(qr,f.qr)!=q::Status::kSuccess||t::InitializeReference(tr,f.tr)!=t::Status::kSuccess||
     q::InitializeLayeredJ2History(f.qr,p,{},f.qh)!=q::Status::kSuccess||
     t::InitializeLayeredJ2History(f.tr,p,{},f.th)!=t::Status::kSuccess)
    throw std::runtime_error("Family preparation failed");
  f.qi.dt=f.ti.dt=1e-6; f.qi.sample_index=f.ti.sample_index=1;
  for(unsigned i=0;i<4;++i) {
    f.qi.position_endpoint[i]=qr.position[i];
    f.qi.velocity_midpoint[i]={2000*qr.position[i].x,-600*qr.position[i].y,0};
  }
  for(unsigned i=0;i<3;++i) {
    f.ti.position[i]=tr.position[i];
    f.ti.velocity[i]={2000*tr.position[i].x,-600*tr.position[i].y,0};
  }
  return f;
}
} // namespace layered_j2_test
