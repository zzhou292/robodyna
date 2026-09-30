#pragma once
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>
namespace pen3_test {
struct Packet {
  std::array<double,15> coordinates{};
  std::array<int,6> flags{1,2,3,3,1,0};
  double gap = 0;
};
struct Case {
  Packet target;
  double margin = 0;
  std::string family;
};
inline Packet Triangle(double scale, double offset, unsigned axis, unsigned kind) {
  Packet p;
  const std::array<std::array<double,3>,5> base{{
      {-1,0,0},{1,0,0},{0,1,0},{0,1,0},{0,.5,.25}}};
  for (unsigned i=0;i<5;++i) for (unsigned c=0;c<3;++c)
    p.coordinates[3*i+(c+axis)%3] = offset + scale*base[i][c];
  if (kind==1) { // Collinear triangle; native denominator floors remain active.
    for (unsigned i : {2u,3u}) p.coordinates[3*i+(1+axis)%3] = offset;
  } else if (kind==2) { // Coincident corners with distinct source IDs.
    for (unsigned i=1;i<4;++i) for (unsigned c=0;c<3;++c)
      p.coordinates[3*i+c]=p.coordinates[c];
  } else if (kind==3) { // Outside the selected triangle, near its first corner.
    p.coordinates[12+axis%3] = offset - 1.25*scale;
    p.coordinates[12+(1+axis)%3] = offset - .125*scale;
  } else if (kind==4) { // Tilt and shear using represented dyadic coefficients.
    for (unsigned i=0;i<5;++i) {
      const double x=p.coordinates[3*i+axis%3]-offset;
      const double y=p.coordinates[3*i+(1+axis)%3]-offset;
      p.coordinates[3*i+(2+axis)%3] += .125*x - .0625*y;
    }
  }
  return p;
}
inline Packet Unrelated(bool triangle) {
  Packet p;
  p.coordinates={100,100,100, 102,100,100, 102,102,100,
      100,102,100, 101,101,103};
  p.flags={11,12,13,14,1,0};
  if (triangle) {
    p.flags[3]=p.flags[2];
    for (unsigned c=0;c<3;++c) p.coordinates[9+c]=p.coordinates[6+c];
  }
  p.gap=.25;
  return p;
}
inline std::vector<Case> Cases() {
  std::vector<Case> result;
  for (double scale : {0x1p-30, 1., 0x1p30})
    for (double offset : {0., 0x1p20})
      for (unsigned axis=0;axis<3;++axis)
        for (unsigned shape=0;shape<5;++shape)
          for (unsigned side=0;side<2;++side) {
            auto packet=Triangle(scale,offset,axis,shape);
            if (side) packet.coordinates[12+(2+axis)%3] -= .5*scale;
            const double exact_gap=.25*scale;
            for (double gap : {0.,std::nextafter(exact_gap,0.),exact_gap,
                    std::nextafter(exact_gap,INFINITY),.5*scale}) {
              packet.gap=gap;
              for (double margin : {0.,.125*scale}) {
                result.push_back({packet,margin,"shape-"+std::to_string(shape)});
              }
            }
          }
  // The native symmetry screen is separate from the cohort geometry branch.
  for (int symmetry : {0,1,2,3,4,5,6,7}) {
    auto p=Triangle(1.,0.,0,0);
    p.flags[4]=0;
    p.flags[5]=symmetry;
    p.gap=.5;
    result.push_back({p,0.,"solid-symmetry"});
  }
  // Deterministic finite stress corpus. No source-ID filtering or translated
  // eligibility selection: only the target is changed, identically in all modes.
  std::uint64_t state=0x912842314578abcduLL;
  for (unsigned sample=0;sample<256;++sample) {
    auto p=Triangle(1.,0.,0,0);
    for (unsigned i=0;i<15;++i) {
      state=state*6364136223846793005uLL+1442695040888963407uLL;
      const double perturbation=double((state>>40)&0xffff)/65536.-.5;
      p.coordinates[i] += perturbation*.25;
    }
    for (unsigned c=0;c<3;++c) p.coordinates[9+c]=p.coordinates[6+c];
    p.gap=.25;
    result.push_back({p,.0625,"finite-perturbed"});
  }
  return result;
}
} // namespace pen3_test
