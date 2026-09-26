// SPDX-License-Identifier: AGPL-3.0-or-later
#include "lib_src/collision/RadiossType25MainGeometry.h"
#ifdef __FAST_MATH__
#error Native geometry consumers require inherited precise arithmetic
#endif
int main() {
  namespace n=tlfea::contact::radioss_type25;
  n::NativeExteriorMainGeometryInput in;
  in.layout=n::ShellLayout::Quad4;
  in.face[0]={-1,-1,0};in.face[1]={1,-1,0};in.face[2]={1,1,0};in.face[3]={-1,1,0};
  for(unsigned i=0;i<8;++i) { in.solid_raw[i]=in.face[i%4];in.solid_raw[i].z=i<4?0.:2.; }
  n::NativeExteriorMainGeometryResult out;
  return n::EvaluateNativeExteriorMainGeometry(in,&out)!=n::CoefficientStatus::Ok ||
    out.area!=4. || out.signed_volume!=8. || !out.reversed || out.source_corner[0]!=3;
}
