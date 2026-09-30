#!/usr/bin/env python3
"""Only namespace/include/declaration adaptation of the frozen 01a9a39 caller."""
from pathlib import Path
import sys

HERE = Path(__file__).resolve().parent

def generated():
    values = (HERE/'reference/ScreenValues.h').read_text()
    values = values.replace('#include "Sources.h"', '#include "lib_src/solvers/cin_timestep/Sources.h"')
    values = values.replace('namespace tl::fea::cin_timestep::detail {',
        'namespace tl::fea::cin_group_test::frozen_values {\nusing namespace cin_timestep;')
    screen = (HERE/'reference/Screen.h').read_text()
    screen = screen.replace('#include "ScreenValues.h"', '')
    screen = screen.replace('namespace tl::fea::cin_timestep {',
        'namespace tl::fea::cin_group_test::frozen_values {')
    screen = screen.replace('detail::', '')
    summary = (HERE/'reference/ScreenSummary.h').read_text()
    summary = summary.replace('#include "../cin_timestep/ScreenValues.h"', '#include "FrozenValues.h"\n#include "lib_src/solvers/cin_advance/Screen.h"')
    summary = summary.replace('#include "../NodalStateLimits.h"', '')
    summary = summary.replace('namespace tl::fea::cin_advance::screen {',
        'namespace tl::fea::cin_group_test::frozen_screen {\nusing cin_advance::Input;')
    a = summary.index('struct Summary {')
    b = summary.index('};', a)+2
    summary = summary[:a]+'using Summary = cin_advance::screen::Summary;'+summary[b:]
    summary = summary.replace('cin_timestep::detail::', 'frozen_values::')
    summary = summary.replace('  Merge(summary,', '  frozen_screen::Merge(summary,')
    summary += '\nnamespace tl::fea::cin_group_test::frozen_screen {\nusing cin_advance::screen::Sources;\ncudaError_t Launch(const Input&, cudaStream_t);\n}\n'
    kernels = (HERE/'reference/Screen.cu').read_text()
    kernels = kernels.replace('#include "Screen.h"', '#include "FrozenSummary.h"')
    kernels = kernels.replace('namespace tl::fea::cin_advance::screen {',
        'namespace tl::fea::cin_group_test::frozen_screen {\nusing cin_advance::NoFailure;')
    kernels = kernels.replace('cin_timestep::detail::', 'frozen_values::')
    # Summary aliases the existing scratch ABI; qualify calls to suppress ADL
    # into the production namespace without changing their expression bodies.
    for name in ('Merge', 'Observe', 'Complete'):
        kernels = kernels.replace(name+'(', 'frozen_screen::'+name+'(')
    owner = (HERE/'reference/ExplicitNodalCinStep.cu').read_text()
    start = owner.index('  for (std::uint32_t g = 0;')
    end = owner.index('  stage = cin::RecoverMotionTrial', start)
    group_body = owner[owner.index('{', start)+1:owner.rfind('}', start, end)]
    motion = '''// Generated exact original one-group body; test-only local status sink.
#pragma once
#include "lib_src/solvers/cin_advance/Groups.h"
namespace tl::fea::cin_group_test {
inline void FrozenMotion(const cin_advance::Input& input, std::uint32_t g) {
  auto* control = input.control;
  const auto* accepted = input.accepted;
  auto* trial = input.trial;
  auto* loads = input.loads;
  const auto groups = input.groups;
  const auto durations = input.durations;
  const auto maximum_angle = input.maximum_angle;
  const auto capture = input.capture;
  const auto n = input.model.node_count;
  const auto Fail = [](nodal_detail::Control* c, NodalStatus status, std::uint32_t node) {
    c->status = status; c->node = node;
    if (status == NodalStatus::StepTooLarge) c->limit.dt = 0;
  };
'''+group_body+'\n}\n}\n'
    owner = owner[:owner.index('cudaError_t FENodalState::Impl::LaunchCinAdvance')]
    owner = owner.replace('namespace tl::fea {', 'namespace tl::fea::cin_group_test {\nusing namespace cin_advance;')
    owner = owner.replace('cudaError_t cin_advance::Launch(', 'cudaError_t LaunchFrozen(')
    owner = owner.replace('cin_timestep::Screen(', 'frozen_values::Screen(')
    owner = owner.replace('screen::', 'frozen_screen::')
    owner += '} // namespace tl::fea::cin_group_test\n'
    return {'FrozenValues.h': values+screen, 'FrozenSummary.h': summary, 'FrozenMotion.h': motion,
        'Frozen.cu': kernels+'\n'+owner}

if __name__ == '__main__':
    for name, data in generated().items():
        if '--check' in sys.argv:
            assert (HERE/name).read_text() == data, name
        else:
            (HERE/name).write_text(data)
