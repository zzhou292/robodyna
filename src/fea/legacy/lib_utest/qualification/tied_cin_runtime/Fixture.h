#pragma once
#include "lib_src/constraints/tied_shell/runtime/CinMotionStage.h"
#include "lib_src/solvers/NodalCinStorage.h"
#include "lib_utest/qualification/tied_cin_attachment/Fixture.h"
#include <limits>

namespace cin_runtime_test {
namespace tied = tl::constraints::tied_shell;
namespace cin = tied::cin;
namespace fea = tl::fea;
struct Fixture {
  cin_test::Fixture source;
  tied::TiedCinAttachmentModel model;
  std::vector<cin::StageRow> rows;
  std::vector<cin::WitnessRange> ranges;
  std::vector<cin::ActiveWitness> witnesses;
  std::vector<std::uint8_t> dependent, flags, fixed;
  std::vector<double> x, velocity, omega, q, load, mass, inertia, stif, stifr;
  std::vector<double> inverse, inverse_j, saved_mass, saved_inertia, entry_j, a, ar;
  std::vector<tied::Patch> patches;
  double dmas = 0;
  Fixture() {
    if (!tied::PrepareCinAttachments(source.post, source.domain, source.Input(), &model)) {
      throw std::runtime_error("Runtime model rejected");
    }
    const auto n = source.domain.node_count();
    const auto count = model.rows().count;
    x.resize(3*n);
    velocity.resize(3*n);
    omega.resize(3*n);
    q.resize(4*n);
    load.resize(6*n);
    mass.resize(n);
    inertia.resize(n);
    inverse.resize(n);
    inverse_j.resize(n);
    stif.resize(n);
    stifr.resize(n);
    entry_j.resize(n);
    dependent.resize(n);
    fixed.resize(n);
    a.resize(3*n);
    ar.resize(3*n);
    saved_mass.resize(count);
    saved_inertia.resize(count);
    patches.resize(count);
    for (std::size_t i = 0; i < n; ++i) {
      const auto p = source.domain.nodes()[i].position;
      x[3*i] = p.x;
      x[3*i+1] = p.y;
      x[3*i+2] = p.z;
      q[4*i] = 1;
      mass[i] = 2+.125*i;
      inertia[i] = .01+.001*i;
      inverse[i] = 1/mass[i];
      inverse_j[i] = 1/inertia[i];
      stif[i] = 4+i;
      stifr[i] = .1+.01*i;
      for (unsigned axis = 0; axis < 3; ++axis) {
        load[axis*n+i] = (axis+1)*(.5+.1*i);
        load[(axis+3)*n+i] = (axis+1)*(.03+.01*i);
        velocity[3*i+axis] = .02*(i+axis);
        a[3*i+axis] = .1*(i+axis);
      }
    }
    for (std::size_t r = 0; r < count; ++r) {
      const auto& mapped = model.rows().data[r];
      cin::StageRow row;
      row.secondary = mapped.secondary_domain_node;
      std::copy(mapped.master_domain_nodes.begin(), mapped.master_domain_nodes.end(), row.masters);
      row.witnesses = {std::uint32_t(r), 1};
      rows.push_back(row);
      ranges.push_back(row.witnesses);
      cin::ActiveWitness witness;
      witness.source_element_id = mapped.master_source.element_id;
      witness.native_parent_index = r;
      witness.family = r ? cin::WitnessFamily::ShellTriangle : cin::WitnessFamily::ShellQuad;
      std::copy(row.masters, row.masters+4, witness.nodes);
      witnesses.push_back(witness);
      dependent[row.secondary] = 1;
      inverse[row.secondary] = 0;
      inverse_j[row.secondary] = 0;
    }
    flags.assign(count, 1);
  }
  cin::StageView View() const {
    return {rows.data(), dependent.data(), std::uint32_t(mass.size()),
      std::uint32_t(rows.size()), std::uint32_t(flags.size())};
  }
  cin::ForceTrial Force() {
    return {x.data(), load.data(), mass.data(), inertia.data(), stif.data(), stifr.data(),
      saved_mass.data(), saved_inertia.data(), &dmas, entry_j.data(), patches.data(), flags.data()};
  }
  cin::MotionTrial Motion() {
    return {patches.data(), velocity.data(), omega.data(), a.data(), ar.data()};
  }
  fea::NodalStateConfig Config() const {
    fea::NodalStateConfig c;
    c.node_count = mass.size();
    c.max_device_bytes = 4u << 20;
    c.fixed_dt = 1e-6;
    c.temporal_scheme = fea::NodalTemporalScheme::StaggeredHalfKickStart;
    return c;
  }
  fea::HostNodalKinematicsView Kinematics() const {
    return {x.data(), velocity.data(), omega.data(), mass.size(), q.data()};
  }
  fea::NodalDofConfig Dofs() const { return {fixed.data(), fixed.data(), inverse_j.data()}; }
  fea::NodalCinStartup Startup() const {
    return {&model, mass.data(), inertia.data(), ranges.data(), witnesses.data(), witnesses.size(), 871};
  }
};
} // namespace cin_runtime_test
