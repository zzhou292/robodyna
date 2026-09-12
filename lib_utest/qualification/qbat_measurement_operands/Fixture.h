// SPDX-License-Identifier: MIT
#pragma once
#include "../qbat_mapped_gather/Fixture.h"
#include "SerialFinalize.h"
#include <algorithm>
#include <vector>

namespace qbat_measurement_test {
namespace g=qbat_gather_test;
namespace fe=tl::fea;
namespace q=fe::qbat;
namespace b=q::batch_detail;
namespace m=q::mapped;
using g::Bits;
// Expand the existing qualified four-parent geometry without another law,
// geometry or owner fixture. Source ordering deliberately cycles its slots.
struct Fixture {
  g::Fixture seed;
  b::Layout layout;
  tl::util::HostArena arena;
  b::Storage* host=nullptr;
  g::Inputs input;
  std::size_t count;
  explicit Fixture(std::size_t parents=4) : count(parents) {
    EXPECT_TRUE(layout.InitializeMapped(count,g::Nodes,0,64u<<20));
    EXPECT_TRUE(arena.Initialize(layout.bytes));
    host=layout.Construct(arena);
    EXPECT_NE(host,nullptr);
    if (!host) return;
    host->model.config=seed.host->model.config;
    host->model.config.element_count=count;
    host->model.mapped=true;
    for (std::size_t node=0; node<g::Nodes; ++node)
      host->model.initial_position[node]=seed.host->model.initial_position[node];
    Reset(2);
  }
  void Reset(unsigned epoch=2) {
    seed.Prepare(epoch);
    input=seed.input;
    for (std::size_t node=0; node<g::Nodes; ++node) {
      for (unsigned axis=0; axis<3; ++axis) {
        input.velocity[3*node+axis]=.125*(1+axis)*(1+node%7);
        input.omega[3*node+axis]=-.25*(1+axis)*(1+node%3);
      }
    }
    host->model.config.usage=q::BatchUsage::CoupledForces;
    for (std::size_t parent=0; parent<count; ++parent) {
      const auto original=parent%g::Parents;
      host->model.element[parent]=seed.host->model.element[original];
      host->model.element[parent].source_parent_id=200+parent;
      host->candidate_status[parent]=q::Status::kSuccess;
      for (unsigned slab=0; slab<2; ++slab)
        host->slab[slab].element[parent]=seed.host->slab[slab].element[original];
    }
  }
  void Stage(unsigned epoch=2) {
    const auto view=input.Prepared(epoch);
    const auto identity=g::Identity(epoch);
    for (std::size_t parent=0; parent<count; ++parent) {
      host->assembly.measurement[parent]=m::PrepareMeasurementParent(
          *host,host->slab[0],host->slab[1],view,identity,parent);
    }
    auto& maximum=host->assembly.maximum[0];
    maximum={0,true};
    for (std::size_t node=0; node<g::Nodes; ++node) {
      double value=0;
      if (!b::NodeDisplacement(host->model,view,node,value)) maximum.valid=false;
      else if (value>maximum.value) maximum.value=value;
    }
  }
  void Compare(unsigned epoch=2) {
    Stage(epoch);
    const auto view=input.Prepared(epoch);
    const auto identity=g::Identity(epoch);
    serial::Finalize(host,&host->slab[0],&host->slab[1],view,identity);
    const auto expected=host->control;
    m::FinalizeMeasurement(*host,view,identity,1);
    EXPECT_EQ(host->control.status,expected.status);
    EXPECT_EQ(host->control.element_status,expected.element_status);
    EXPECT_EQ(host->control.element,expected.element);
    EXPECT_EQ(host->control.node,expected.node);
    EXPECT_TRUE(b::SameDiagnostics(host->control.diagnostics,expected.diagnostics));
  }
};
inline void Fault(Fixture& f,unsigned fault) {
  auto* rows=f.host->slab[1].element;
  const auto last=f.count-1;
  const auto nan=std::numeric_limits<double>::quiet_NaN();
  const auto huge=std::numeric_limits<double>::max();
  if (fault==1) rows[last].point[3].force_volume_m3=nan;
  if (fault==2) f.input.endpoint[3*(g::Nodes-1)]=nan;
  if (fault==3) {
    f.input.endpoint[0]=nan;
    f.input.endpoint[3*(g::Nodes-1)]=1e3;
  }
  if (fault==4 || fault==5) {
    rows[0].point[3].force_volume_m3=nan;
    f.host->candidate_status[last]=q::Status::kInvalidInput;
    if (fault==5) f.host->candidate_status[1]=q::Status::kInvalidReference;
  }
  if (fault==6 || fault==7) {
    rows[0].history.internal_work_j[0]=huge;
    rows[1].history.internal_work_j[0]=huge;
    if (fault==7) rows[last].point[3].force_volume_m3=nan;
  }
  if (fault==8) {
    // Finite inputs can overflow the derived work term. The later invalid row
    // must still expose the same accumulated prefix, not an early leaf error.
    f.host->slab[0].element[0].internal_force_n[0].x=huge;
    f.input.velocity[3*f.host->model.element[0].nodes[0]]=huge;
    rows[last].point[3].force_volume_m3=nan;
  }
  if (fault==9) {
    auto* raw=reinterpret_cast<unsigned char*>(&rows[last].history.element_active);
    *raw=2;
  }
  if (fault==10) {
    g::Remove(rows[last]);
    f.input.omega[0]=nan; // Preserve even zero-couple multiplication semantics.
  }
}
} // namespace qbat_measurement_test
