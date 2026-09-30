#pragma once

#include "Q4ContactIntegrationTypes.h"

namespace tlfea::contact::q4_bounds {

using Interval = Q4IntegralInterval;

// Narrow IEEE-binary64 enclosures for this contact integral. RN arithmetic and
// sqrt, no reassociation/fast-math/FTZ. Exact zero is preserved; a nonzero
// product lost to underflow is rejected. Lower positive bounds may round to 0.
TL_SURFACE_HD inline bool Finite(Interval a) {
  return IsFinite(a.lower) && IsFinite(a.upper) && a.lower <= a.upper;
}
TL_SURFACE_HD inline bool Nonnegative(Interval a) { return Finite(a) && a.lower >= 0; }
TL_SURFACE_HD inline bool Round(double value, bool upper, double* output) {
  if (!IsFinite(value)) return false;
  if (upper && value >= 0) return mass_detail::Upper(value,output);
  *output = value == 0 ? 0 : ::nextafter(value, upper ? HUGE_VAL : -HUGE_VAL);
  return IsFinite(*output);
}
TL_SURFACE_HD inline bool AddScalar(double a, double b, bool upper, double* output) {
  if (!IsFinite(a) || !IsFinite(b)) return false;
  if (a == 0) { *output=b; return true; }
  if (b == 0) { *output=a; return true; }
  // Error-free two-sum identifies exact dyadic restrictions and cancellation.
  // Padding every exact +/- pair would manufacture uncertain saddle seams.
  const double sum=a+b;
  if (!IsFinite(sum)) return false;
  const double b_virtual=sum-a, a_virtual=sum-b_virtual;
  const double b_round=b-b_virtual, a_round=a-a_virtual;
  const double error=a_round+b_round;
  if (!IsFinite(b_virtual) || !IsFinite(a_virtual) || !IsFinite(error)) return false;
  if (error == 0 || (upper ? error < 0 : error > 0)) { *output=sum; return true; }
  return Round(sum,upper,output);
}
TL_SURFACE_HD inline bool MultiplyScalar(double a, double b, bool upper, double* output) {
  if (!IsFinite(a) || !IsFinite(b)) return false;
  if (a == 0 || b == 0) { *output=0; return true; }
  if (a == 1) { *output=b; return true; }
  if (b == 1) { *output=a; return true; }
  if (a == -1) { *output=-b; return true; }
  if (b == -1) { *output=-a; return true; }
  const double value=a*b;
  if (!IsFinite(value) || value == 0) return false;
  int exponent=0;
  // An exact subnormal product may round UP onto DBL_MIN; that boundary is
  // not evidence of exact scaling. Strictly above it, power-of-two scaling
  // into the normal range cannot have discarded significand bits.
  if (::fabs(value) > DBL_MIN &&
      (::fabs(::frexp(a,&exponent)) == .5 || ::fabs(::frexp(b,&exponent)) == .5)) {
    *output=value; return true;  // Exact power-of-two scaling into the normal range.
  }
  return Round(value,upper,output);
}
TL_SURFACE_HD inline bool Add(Interval a, Interval b, Interval* output) {
  Interval next;
  if (!Finite(a) || !Finite(b) || !AddScalar(a.lower,b.lower,false,&next.lower) ||
      !AddScalar(a.upper,b.upper,true,&next.upper)) return false;
  *output=next; return true;
}
TL_SURFACE_HD inline bool Difference(double a, double b, Interval* output) {
  Interval next;
  if (!AddScalar(a,-b,false,&next.lower) || !AddScalar(a,-b,true,&next.upper)) return false;
  *output=next; return true;
}
TL_SURFACE_HD inline bool Scale(Interval a, double positive, Interval* output) {
  Interval next;
  if (!Finite(a) || !IsFinite(positive) || positive < 0 ||
      !MultiplyScalar(a.lower,positive,false,&next.lower) ||
      !MultiplyScalar(a.upper,positive,true,&next.upper)) return false;
  *output=next; return true;
}
TL_SURFACE_HD inline bool MultiplyPositive(Interval a, Interval b, Interval* output) {
  Interval next;
  if (!Nonnegative(a) || !Nonnegative(b) ||
      !MultiplyScalar(a.lower,b.lower,false,&next.lower) ||
      !MultiplyScalar(a.upper,b.upper,true,&next.upper)) return false;
  *output=next; return true;
}
TL_SURFACE_HD inline bool DividePositive(Interval a, double divisor, Interval* output) {
  if (!Nonnegative(a) || !IsFinite(divisor) || divisor <= 0) return false;
  if (divisor == 1) { *output=a; return true; }
  Interval next;
  const double lower=a.lower/divisor, upper=a.upper/divisor;
  if ((a.lower > 0 && lower == 0) || (a.upper > 0 && upper == 0) ||
      !Round(lower,false,&next.lower) || !Round(upper,true,&next.upper)) return false;
  *output=next; return true;
}
TL_SURFACE_HD inline bool AbsoluteDifferenceUpper(double a, double b, double* output) {
  if (!IsFinite(a) || !IsFinite(b)) return false;
  if (a == b) { *output=0; return true; }
  const double difference=::fabs(a-b);
  return difference != 0 && Round(difference,true,output);
}
TL_SURFACE_HD inline bool Certify(double value, Interval truth, Q4CertifiedIntegral* output) {
  double from_lower=0,from_upper=0;
  if (!Nonnegative(truth) || !IsFinite(value) || value < 0 ||
      !AbsoluteDifferenceUpper(value,truth.lower,&from_lower) ||
      !AbsoluteDifferenceUpper(value,truth.upper,&from_upper)) return false;
  *output={value,truth.lower,truth.upper,from_lower > from_upper ? from_lower : from_upper};
  return true;
}

// Counterclockwise Chrono Q4 order (+,+),(-,+),(-,-),(+,-). On cell corners at
// depth <=16, coordinates and N values are dyadic with <=34 significant bits;
// EvaluateQ4Shape's additions/products are exact in binary64 there. This fact
// is used only for corner bounds, never for rounded Gauss abscissae.
TL_SURFACE_HD inline bool CellCorners(const Q4IntegrationCell& cell, double shape[4][4]) {
  if (cell.depth > MaxQ4IntegrationDepth || cell.column >= (1u << cell.depth) ||
      cell.row >= (1u << cell.depth)) return false;
  const double side=::ldexp(1.0,-static_cast<int>(cell.depth));
  const double u[2]={-1+2*cell.column*side,-1+2*(cell.column+1)*side};
  const double v[2]={-1+2*cell.row*side,-1+2*(cell.row+1)*side};
  const unsigned u_side[4]={1,0,0,1},v_side[4]={1,1,0,0};
  for (unsigned corner=0;corner<4;++corner)
    if (EvaluateQ4Shape(u[u_side[corner]],v[v_side[corner]],shape[corner]) != Status::kOk) return false;
  return true;
}
TL_SURFACE_HD inline bool Restrict(const Interval parent[4], const double shape[4], Interval* output) {
  Interval result;
  for (unsigned i=0;i<4;++i) {
    Interval term;
    if (!Scale(parent[i],shape[i],&term) || !Add(result,term,&result)) return false;
  }
  *output=result; return true;
}
TL_SURFACE_HD inline bool CellArea(double parent_area, std::uint32_t depth, Interval* output) {
  return parent_area > 0 && depth <= MaxQ4IntegrationDepth &&
         Scale({parent_area,parent_area},::ldexp(1.0,-2*static_cast<int>(depth)),output);
}

// Exact real bilinear product moments: A/36 times the positive integer matrix.
// No rounded Gauss node is involved in this independent continuum enclosure.
TL_SURFACE_HD inline bool ActiveMoments(const Interval gaps[4], const double shape[4][4],
                                      Interval area, double stiffness, Interval output[5]) {
  const unsigned coefficient[4][4]={{4,2,1,2},{2,4,2,1},{1,2,4,2},{2,1,2,4}};
  Interval factor;
  if (!DividePositive(area,36,&factor) || !Scale(factor,stiffness,&factor)) return false;
  Interval sums[5]{};
  for (unsigned i=0;i<4;++i) for (unsigned j=0;j<4;++j) {
    if (!Nonnegative(gaps[i]) || !Nonnegative(gaps[j])) return false;
    for (unsigned node=0;node<4;++node) {
      Interval term;
      if (!Scale(gaps[j],shape[i][node]*coefficient[i][j],&term) ||
          !Add(sums[node],term,&sums[node])) return false;
    }
    Interval energy_term;
    if (!MultiplyPositive(gaps[i],gaps[j],&energy_term) ||
        !Scale(energy_term,coefficient[i][j],&energy_term) ||
        !Add(sums[4],energy_term,&sums[4])) return false;
  }
  for (unsigned i=0;i<5;++i)
    if (!MultiplyPositive(sums[i],factor,&output[i])) return false;
  return Scale(output[4],.5,&output[4]);
}
TL_SURFACE_HD inline bool MixedBounds(double penetration, const double shape[4][4],
                                    Interval area, double stiffness, Interval output[5]) {
  if (!IsFinite(penetration) || penetration <= 0) return false;
  Interval pressure;
  if (!Scale({penetration,penetration},stiffness,&pressure)) return false;
  for (unsigned node=0;node<4;++node) {
    double mean=0;
    for (unsigned corner=0;corner<4;++corner) mean+=.25*shape[corner][node];
    Interval weight,result;
    if (!Scale(area,mean,&weight) || !MultiplyPositive(weight,pressure,&result)) return false;
    output[node]={0,result.upper};
  }
  Interval energy;
  if (!MultiplyPositive(area,pressure,&energy) || !Scale(energy,penetration,&energy) ||
      !Scale(energy,.5,&energy)) return false;
  output[4]={0,energy.upper}; return true;
}

}  // namespace tlfea::contact::q4_bounds
