// SPDX-License-Identifier: AGPL-3.0-or-later
#include "NativeOracle.h"
#include "../radioss_type25_nodal_contributions/NativeOracle.h"
#include "../radioss_type25_coefficients/NativeOracle.h"
#include "../beam18_reference/NativeOracle.h"
#include "lib_src/math/ScalarBits.h"
#include <algorithm>
#include <set>
#include <stdexcept>
namespace type25_seed_test {
extern "C" void rd_seed_gather(int,int,int,int,int,int,int,
    const int*,const int*,const int*,const int*,const int*,const int*,
    const double*,const double*,const double*,const double*,const double*,const double*,
    const double*,const double*,const double*,double*,double*,double*,int*,double*);
extern "C" void rd_seed_joint(const double*,double*,double*);
extern "C" void rd_seed_initialize(double*,double*);
namespace {
void Require(bool valid,const char* message) {
  if (!valid) throw std::invalid_argument(message);
}
struct Table {
  unsigned width;
  std::vector<int> values;
  std::set<int> ids;
  explicit Table(unsigned columns) : width(columns) {}
  void Add(int eid,const std::uint32_t* nodes,unsigned count,std::size_t domain,int kind=0) {
    Require(eid>0 && ids.insert(eid).second,"Native seed duplicate/nonpositive EID");
    const auto first=values.size();
    values.resize(first+width);
    auto* row=values.data()+first;
    row[0]=int(first/width)+1;
    for(unsigned j=0;j<count;++j) {
      Require(nodes[j]<domain,"Native seed node exceeds domain");
      row[j+1]=int(nodes[j])+1;
    }
    if(kind)row[4]=kind;
    row[width-1]=eid;
  }
  int Count() const {return int(values.size()/width);}
  const int* Data() {if(values.empty())values.resize(width);return values.data();}
};
const double* Data(std::vector<double>& values) {
  if(values.empty())values.push_back(0.);
  return values.data();
}
double Spring(const Direct& row) {
  if(row.kind==DirectKind::Type45) {
    double result=73,observed=0;
    rd_seed_joint(&row.joint_kn,&result,&observed);
    Require(tl::math::SameScalarBits(observed,row.joint_kn),"Native joint Kn storage witness differs");
    Require(tl::math::SameScalarBits(result,0.),"Native TYPE45 contact coefficient is not+0");
    return result;
  }
  Require(row.kind==DirectKind::Type13||row.kind==DirectKind::Type25,"Unselected native spring");
  // The input is the explicitly prepared XL boundary, as in the qualified
  // scalar reference. Coordinate/length preparation is not repeated here.
  const double length=row.spring.length_mode>0?row.spring.geometric_length:1.;
  return type25_contribution_test::OracleSpringPrepared(row.spring,length);
}
}
NativeObservation Oracle(const Case& input) {
  Require(input.nodes && input.nodes<=1024 && input.solids.size()<=256 &&
      input.direct.size()<=256 && input.shells.size()<=256,"Native seed qualification cap");
  Table solids(11),quads(7),triangles(6),trusses(5),beams(6),springs(6);
  std::vector<double> vns,bns,youngq,thkq,youngt,thkt,stt,stp,str;
  std::array<double,8> initial_volume,initial_bulk;
  rd_seed_initialize(initial_volume.data(),initial_bulk.data());
  for(unsigned slot=0;slot<8;++slot) {
    Require(tl::math::SameScalarBits(initial_volume[slot],0.) &&
        tl::math::SameScalarBits(initial_bulk[slot],0.),"Native LECTUR initializer differs");
  }
  for(const auto& row:input.solids) {
    solids.Add(row.eid,row.nodes.data(),8,input.nodes);
    // LECTUR's original VNS/BNS initial state is+0. SBULK3 retains that value
    // in Penta slots4/8; the native observation keeps all8 raw occurrences.
    const auto native=type25_contribution_test::OracleSolid(row.input,initial_volume[0]);
    if(row.input.kind==n::SolidNodalKind::Penta6)for(unsigned slot:{3u,7u}) {
      Require(tl::math::SameScalarBits(native.volume[slot],initial_volume[slot]) &&
          tl::math::SameScalarBits(native.bulk_volume[slot],initial_bulk[slot]),
          "Penta unassigned slots did not preserve actual native initialization");
    }
    vns.insert(vns.end(),native.volume.begin(),native.volume.end());
    bns.insert(bns.end(),native.bulk_volume.begin(),native.bulk_volume.end());
  }
  for(const auto& row:input.shells) {
    Require(row.source_element_id>0 && row.source_element_id<=INT32_MAX,"Native shell EID range");
    if(row.layout==n::ShellLayout::Quad4) {
      quads.Add(int(row.source_element_id),row.nodes,4,input.nodes);
      youngq.push_back(row.young);thkq.push_back(row.structural_thickness);
    } else {
      Require(row.layout==n::ShellLayout::Triangle3 && row.nodes[3]==row.nodes[2],"Native shell layout");
      triangles.Add(int(row.source_element_id),row.nodes,3,input.nodes);
      youngt.push_back(row.young);thkt.push_back(row.structural_thickness);
    }
  }
  for(const auto& row:input.direct) {
    if(row.kind==DirectKind::Truss) {
      trusses.Add(row.eid,row.nodes.data(),2,input.nodes);
      stt.push_back(row.truss_stiffness);
    } else if(row.kind==DirectKind::Beam18) {
      beams.Add(row.eid,row.nodes.data(),2,input.nodes);
      const auto native=beam18_test::Native(row.beam_input);
      Require(!native.status,"Native beam source packet failed");
      stp.push_back(native.values[24]); // Actual PMASS STP, not structural STI.
    } else {
      const int kind=row.kind==DirectKind::Type13?13:(row.kind==DirectKind::Type25?25:45);
      springs.Add(row.eid,row.nodes.data(),2,input.nodes,kind);
      str.push_back(Spring(row));
    }
  }
  const int ns=solids.Count(),nq=quads.Count(),nt=triangles.Count();
  const int ntruss=trusses.Count(),nbeam=beams.Count(),nspring=springs.Count();
  NativeObservation out;
  std::vector<double> volume(input.nodes),bulk(input.nodes),stiffness(input.nodes);
  out.young_thickness.resize(input.nodes);out.shell_incidence.resize(input.nodes);
  rd_seed_gather(int(input.nodes),ns,nq,nt,ntruss,nbeam,nspring,
      solids.Data(),quads.Data(),triangles.Data(),trusses.Data(),beams.Data(),springs.Data(),
      Data(vns),Data(bns),Data(youngq),Data(thkq),Data(youngt),Data(thkt),Data(stt),Data(stp),Data(str),
      volume.data(),bulk.data(),out.young_thickness.data(),out.shell_incidence.data(),stiffness.data());
  out.seeds.reserve(input.nodes);out.finalized.reserve(input.nodes);
  for(std::size_t i=0;i<input.nodes;++i) {
    out.seeds.push_back({volume[i],bulk[i],stiffness[i]});
    const n::NativeAccumulatedNodalCoefficients values{volume[i],bulk[i],out.young_thickness[i],
        out.shell_incidence[i],stiffness[i]};
    out.finalized.push_back(type25_coefficient_test::Oracle(values));
  }
  return out;
}
} // namespace type25_seed_test
