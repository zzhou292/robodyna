// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "lib_utest/qualification/extended_solid_resident/Fixture.h"
#include "lib_src/elements/solids/resident/Storage.h"
#include "FrozenMeasure.h"
#include <cfloat>
#include <memory>
#include <vector>

namespace solid_validation_test {
namespace fe = tl::fea;
namespace s = fe::solids;
namespace d = s::batch_detail;
namespace old = d::frozen;

struct HostRig {
  extended_resident_test::Fixture source;
  s::Model model;
  s::BatchConfig config;
  d::ArenaLayout layout;
  tl::util::HostArena arena;
  d::Storage state;

  bool Initialize(int foam_flag = 1, bool analytic44 = false) {
    source.foam_input.hysteresis = foam_flag == 1 ? 0 : 1;
    source.PrepareFoam();
    if (analytic44) {
      namespace law = tl::material::law44::solid;
      const auto material = source.input44[0].material.material;
      if (law::PrepareAnalytic(material, {270e6, 350e6, 1, 0, 0},
          source.input44[0].material) != law::Status::Ok) return false;
    }
    const auto domain = source.Domain();
    if (!model.Initialize(domain, source.Input())) return false;
    config = extended_resident_test::Config(domain.node_count());
    if (!d::Plan(config, model, layout) || !arena.Initialize(layout.bytes) ||
        !d::BuildUpload(config, model, arena, layout, state)) return false;
    return InitializeFamily<d::Traits18>() && InitializeFamily<d::Traits24>() &&
        InitializeFamily<d::Traits6z>() && InitializeFamily<d::Traits18Law44>() &&
        InitializeFamily<d::Traits18Law90>();
  }

  template<class Traits> bool InitializeFamily() {
    auto& f = d::FamilyStorage<Traits>(state);
    for (std::size_t p = 0; p < f.count; ++p) {
      const auto& parent = f.parents[p];
      const auto& material = d::MaterialAt<Traits>(state, parent.material_index);
      int status = 0;
      if constexpr (std::is_same_v<Traits, d::Traits18>) {
        auto scratch = std::make_unique<d::Scratch18>();
        status = d::InitializeState18(parent, material, {}, *scratch, f.slab[0][p]);
      } else if constexpr (std::is_same_v<Traits, d::Traits18Law44> ||
                           std::is_same_v<Traits, d::Traits18Law90>) {
        auto scratch = std::make_unique<d::ExtendedScratch<Traits>>();
        status = d::InitializeExtendedState<Traits>(parent, material, {}, *scratch, f.slab[0][p]);
      } else {
        status = d::InitializeState<Traits>(parent, material, {}, f.slab[0][p]);
      }
      if (status != 0) return false;
      f.slab[1][p] = f.slab[0][p];
      f.status[p] = 0;
      f.result_valid[p] = 255;
    }
    return true;
  }
};

template<class Traits> void Validate(d::Storage& state, unsigned trial, double time,
    std::uint64_t epoch) {
  auto& f = d::FamilyStorage<Traits>(state);
  for (std::size_t p = 0; p < f.count; ++p)
    f.result_valid[p] = d::CheckParentResult<Traits>(state, trial, p, time, epoch);
}
inline void ValidateAll(d::Storage& state, unsigned trial, double time, std::uint64_t epoch) {
  Validate<d::Traits18>(state, trial, time, epoch);
  Validate<d::Traits24>(state, trial, time, epoch);
  Validate<d::Traits6z>(state, trial, time, epoch);
  Validate<d::Traits18Law44>(state, trial, time, epoch);
  Validate<d::Traits18Law90>(state, trial, time, epoch);
}

inline void Reset(d::Storage& state, double time = 0, std::uint64_t epoch = 0) {
  state.control = {};
  state.control.diagnostics.time = time;
  state.control.diagnostics.epoch = epoch;
  state.control.diagnostics.minimum_native_dt_s = DBL_MAX;
}
inline void SameControl(const d::Control& a, const d::Control& b) {
  EXPECT_EQ(a.status, b.status);
  EXPECT_EQ(a.family, b.family);
  EXPECT_EQ(a.parent, b.parent);
  EXPECT_EQ(a.node, b.node);
  EXPECT_EQ(a.element_status, b.element_status);
  EXPECT_TRUE(d::SameDiagnostics(a.diagnostics, b.diagnostics));
}
inline bool MeasureAll(d::Storage& state, bool frozen, unsigned accepted = 0,
    unsigned trial = 1, const fe::NodalPreparedView* view = nullptr) {
  if (frozen)
    return old::MeasureFamily<d::Traits18>(state, accepted, trial, 0, view) &&
        old::MeasureFamily<d::Traits24>(state, accepted, trial, 1, view) &&
        old::MeasureFamily<d::Traits6z>(state, accepted, trial, 2, view) &&
        old::MeasureFamily<d::Traits18Law44>(state, accepted, trial, 3, view) &&
        old::MeasureFamily<d::Traits18Law90>(state, accepted, trial, 4, view);
  return d::MeasureValidatedFamily<d::Traits18>(state, accepted, trial, 0, view) &&
      d::MeasureValidatedFamily<d::Traits24>(state, accepted, trial, 1, view) &&
      d::MeasureValidatedFamily<d::Traits6z>(state, accepted, trial, 2, view) &&
      d::MeasureValidatedFamily<d::Traits18Law44>(state, accepted, trial, 3, view) &&
      d::MeasureValidatedFamily<d::Traits18Law90>(state, accepted, trial, 4, view);
}

template<class Traits> struct Rows {
  std::vector<typename Traits::Parent> parents;
  std::vector<d::State<Traits>> accepted, trial;
  std::vector<int> status;
  std::vector<std::uint8_t> flags;
  Rows(d::Storage& state, std::size_t count) {
    auto& f = d::FamilyStorage<Traits>(state);
    parents.assign(count, f.parents[0]);
    accepted.assign(count, f.slab[0][0]);
    trial = accepted;
    status.assign(count, 0);
    flags.assign(count, 255);
    f = {parents.data(), {accepted.data(), trial.data()}, status.data(), count, flags.data()};
  }
};
} // namespace solid_validation_test
