#include "PlacementNativeSupport.h"
namespace placement_test {
extern "C" void placement_tab1_caller(int,const double*,const double*,const double*,const double*,
    const double*,const double*,const double*,const double*,const double*,const double*,
    double*,double*,double*,double*,double*,double*,double*,double*,double*,double*,int*);
void NativePlacementStep(const sec::ShellLayeredJ2Input& in,double time,double area,double dm,
    NativeState& n,NativeTrace& trace) {
  const double basic[]{2500,70e9,.22,in.transverse_shear_modulus};
  const double linear[]{30e6,1e9},rate[]{0,1,10000},table[]{-.3,0,.3,.015};
  const double reference=n.thickness;
  placement_tab1_caller(NativeIpos(in.placement),basic,linear,rate,table,&in.dt,&time,
      in.strain_curvature_increment,&reference,&area,&dm,n.points.data(),n.failures.data(),
      &n.parent,n.material.data(),n.stress.data(),n.moment.data(),&n.thickness,n.work.data(),
      trace.point_values.data(),trace.diagnostics.data(),&trace.removed);
}
} // namespace placement_test
