// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../../math/Fixed3.h"
#include <cstddef>

#if defined(__CUDACC__)
#define TL_TYPE13_HD __host__ __device__
#else
#define TL_TYPE13_HD
#endif

namespace tl::fea::type13 {
using tl::math::Vec3;
using tl::math::Matrix3;
constexpr unsigned ChannelCount=6,CurveCount=4,CurvePoints=5;
enum class Status { Success,InvalidInput,UnsupportedScope,NonfiniteResult,DegenerateGeometry };
enum class FrameBranch : unsigned { ThirdNode=0,SkewY=1,SkewX=2 };
struct WorkingUnits { double mass_to_kg=0,length_to_m=0,time_to_s=0; };
struct CurvePoint { double x=0,y=0; };
struct CurveView { const CurvePoint* points=nullptr;std::size_t count=0; };
struct Curve { CurvePoint points[CurvePoints]{}; };

// Resolved native Ileng=1 values. Translation x is strain and y is force;
// rotation x is angle/native-length and y is moment. Six independent channels
// refer to four owned curves. No parser/default resolution belongs here.
struct ChannelInput {
  unsigned curve_index=CurveCount;
  double stiffness=0,ordinate_scale=0,abscissa_scale=0,damping=0;
  double failure_negative=0,failure_positive=0,failure_weight=0,failure_exponent=0;
  int hysteresis=0;
};
struct Controls {
  int length_normalized=0,coupled_failure=0,force_failure=0,sensor=0,rate_failure=0;
};
struct PropertyInput {
  WorkingUnits units{};
  double mass_per_length=0,inertia_per_length=0;
  Controls controls{};
  ChannelInput channels[ChannelCount]{};
  CurveView curves[CurveCount]{};
};
struct Channel {
  ChannelInput declaration{};
  double native_stiffness=0; // RKINI3 maximum of declared K and scaled slopes.
  double stiffness_si=0; // N for translation; N*m^2/rad for rotation.
};

class Property {
 public:
  TL_TYPE13_HD bool initialized() const { return initialized_; }
  TL_TYPE13_HD WorkingUnits units() const { return units_; }
  TL_TYPE13_HD double mass_per_length() const { return mass_per_length_; }
  TL_TYPE13_HD double inertia_per_length() const { return inertia_per_length_; }
  TL_TYPE13_HD double added_inertia_per_length() const { return added_inertia_per_length_; }
  TL_TYPE13_HD const Channel& channel(unsigned i) const { return channels_[i]; }
  TL_TYPE13_HD const Curve& curve(unsigned i) const { return curves_[i]; }
 private:
  WorkingUnits units_{};
  double mass_per_length_=0,inertia_per_length_=0,added_inertia_per_length_=0;
  Channel channels_[ChannelCount]{};
  Curve curves_[CurveCount]{};
  bool initialized_=false;
  friend TL_TYPE13_HD Status InitializeProperty(const PropertyInput&,Property&);
};

// Exact working-unit coordinates, including the immutable orientation node.
// N3 is never an endpoint and receives no coefficient in this value adapter.
struct ReferenceInput {
  Vec3 position[3]{};
  Vec3 skew_x{1,0,0},skew_y{0,1,0};
  double coordinate_noise=0; // Resolved XALEA; no default/random generator.
  unsigned endpoint_release[4]{};
};
struct Reference {
  Vec3 position_m[2]{};
  Matrix3 axes{};
  double length_native=0,length_m=0,third_node_alignment=0;
  FrameBranch branch=FrameBranch::ThirdNode;
};
struct EndpointCoefficients {
  double mass_kg=0,isotropic_inertia_kg_m2=0,added_inertia_kg_m2=0;
};
struct Startup {
  Reference reference{};
  EndpointCoefficients endpoint{}; // Identical contribution at N1 and N2.
};
} // namespace tl::fea::type13
