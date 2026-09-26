// SPDX-License-Identifier: AGPL-3.0-or-later
#include "lib_src/collision/RadiossType25Coefficients.h"
int main(){namespace n=tlfea::contact::radioss_type25;
  n::NativeInternalSolidMainCoefficientInput in;in.first.face=n::MainFaceKind::Internal;in.first.layout=n::SolidLayout::EightSlot;
  in.first.area=2;in.first.volume=3;in.first.bulk=4;in.second_fill=1;in.second_bulk=5;in.second_volume=6;
  n::NativeSolidMainCoefficientResult out;
  return n::EvaluateNativeInternalSolidMainCoefficient(in,&out)==n::CoefficientStatus::Ok?0:1;
}
