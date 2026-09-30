#include "lib_src/elements/qeph/QephStartup.h"
#include "lib_src/elements/t3/T3Startup.h"
#include "lib_src/elements/qeph/QephForce.h"
#include "lib_src/elements/t3/T3Force.h"
int main() {
  namespace q=tl::fea::qeph;namespace t=tl::fea::t3;
  const tl::fea::ShellGlobalLaw1Profile profile{tl::fea::ShellLaw1Thickness::Accepted,.001};
  q::ReferenceInput qi;qi.position[0]={0,0,0};qi.position[1]={.02,0,0};
  qi.position[2]={.02,.02,0};qi.position[3]={0,.02,0};
  for(unsigned i=0;i<4;++i)qi.node_ids[i]=i+1;
  q::ReferenceData qr;q::History qh;q::PrescribedInterval qp;q::ForceTrial qo;
  if(q::InitializeReference(qi,qr)!=q::Status::kSuccess||q::InitializeHistory(qr,{},qh)!=q::Status::kSuccess)return 1;
  for(unsigned i=0;i<4;++i)qp.position_endpoint[i]=qi.position[i];qp.dt=1e-6;qp.sample_index=1;
  if(q::EvaluateGlobalLaw1Force(profile,qr,qh,qp,qo)!=q::Status::kSuccess)return 2;
  t::ReferenceInput ti;for(unsigned i=0;i<3;++i){ti.position[i]=qi.position[i];ti.node_ids[i]=i+1;}
  t::ReferenceData tr;t::History th;t::PrescribedInterval tp;t::ForceTrial to;
  if(t::InitializeReference(ti,tr)!=t::Status::kSuccess||t::InitializeHistory(tr,{},th)!=t::Status::kSuccess)return 3;
  for(unsigned i=0;i<3;++i)tp.position[i]=ti.position[i];tp.dt=1e-6;tp.sample_index=1;
  return t::EvaluateGlobalLaw1Force(profile,tr,th,tp,to)==t::Status::kSuccess?0:4;
}
