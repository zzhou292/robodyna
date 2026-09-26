#include "lib_src/collision/RadiossType25MainGeometry.h"
int main(){namespace n=tlfea::contact::radioss_type25;
 n::NativeExteriorMainGeometryInput in;in.layout=n::ShellLayout::Quad4;
 in.face[1]={1,0,0};in.face[2]={1,1,0};in.face[3]={0,1,0};
 n::NativeInternalMainGeometryResult out;double volume=1;
 return n::EvaluateNativeInternalMainGeometry(in,&out)!=n::CoefficientStatus::Ok||
 n::EvaluateNativeEightSlotReaderVolume(in.solid_raw,&volume)!=n::CoefficientStatus::Ok;}
