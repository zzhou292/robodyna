#include "lib_src/elements/beam18/PropertyArea.h"
int main() {
  double area=-1.;
  return tl::fea::beam18::EvaluateCircularPropertyArea(1.,&area)!=tl::fea::beam18::Status::Success || !(area>0.);
}
