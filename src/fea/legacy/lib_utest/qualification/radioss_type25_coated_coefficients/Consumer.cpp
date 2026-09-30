// SPDX-License-Identifier: AGPL-3.0-or-later
#include "lib_src/collision/RadiossType25Coefficients.h"
int main() {
  namespace n=tlfea::contact::radioss_type25;
  n::NativeCoatedMainCoefficientInput in;
  in.shell.face=n::MainFaceKind::Coating;in.shell.layout=n::ShellLayout::Quad4;
  in.shell.property_type=1;in.shell.property_thickness=1;in.shell.young=5;
  in.solid.face=n::MainFaceKind::OrdinaryExterior;in.solid.layout=n::SolidLayout::EightSlot;
  in.solid.area=1;in.solid.volume=1;in.solid.bulk=7;
  n::NativeCoatedMainCoefficientResult out;
  return n::EvaluateNativeCoatedMainCoefficient(in,&out)!=n::CoefficientStatus::Ok ||
      out.primary_stiffness!=7 || out.partner_stiffness!=5 || out.solid_characteristic_length!=1;
}
