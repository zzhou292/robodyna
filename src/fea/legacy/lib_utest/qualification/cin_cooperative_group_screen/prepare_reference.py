#!/usr/bin/env python3
"""Adapt complete frozen 21729f3 screen/member/capture bodies by names only."""
from pathlib import Path
import sys
HERE = Path(__file__).resolve().parent

def generated():
    def read(name):
        return (HERE/'reference'/name).read_text()
    rigid = read('Rigid.h').replace('#include "Ordinary.h"',
        '#include "lib_src/solvers/cin_timestep/Ordinary.h"')
    rigid = rigid.replace('namespace tl::fea::cin_timestep {',
        'namespace tl::fea::cooperative_test::frozen {\nusing namespace cin_timestep;')
    values = read('ScreenValues.h').replace('#include "Sources.h"',
        '#include "lib_src/solvers/cin_timestep/Sources.h"\n#include "FrozenRigid.h"')
    values = values.replace('namespace tl::fea::cin_timestep::detail {',
        'namespace tl::fea::cooperative_test::frozen {\nusing namespace cin_timestep;')
    values = values.replace('!AddRigidMemberTrace(', '!frozen::AddRigidMemberTrace(')
    values = values.replace('!RigidTraceLimit(', '!frozen::RigidTraceLimit(')
    screen = read('Screen.h').replace('#include "ScreenValues.h"', '#include "FrozenValues.h"')
    screen = screen.replace('namespace tl::fea::cin_timestep {',
        'namespace tl::fea::cooperative_test::frozen {')
    screen = screen.replace('detail::', 'frozen::')
    summary = read('ScreenSummary.h').replace('#include "../cin_timestep/ScreenValues.h"',
        '#include "FrozenValues.h"\n#include "lib_src/solvers/cin_advance/Screen.h"')
    summary = summary.replace('#include "../NodalStateLimits.h"', '')
    summary = summary.replace('namespace tl::fea::cin_advance::screen {',
        'namespace tl::fea::cooperative_test::frozen_screen {\nusing cin_advance::Input;')
    a = summary.index('struct Summary {')
    b = summary.index('};', a)+2
    summary = summary[:a]+'using Summary = cin_advance::screen::Summary;'+summary[b:]
    summary = summary.replace('cin_timestep::detail::', 'frozen::')
    summary = summary.replace('  Merge(summary,', '  frozen_screen::Merge(summary,')
    summary += '\nnamespace tl::fea::cooperative_test::frozen_screen {\nusing cin_advance::screen::Sources;\ncudaError_t Launch(const Input&, cudaStream_t);\n}\n'
    groups = read('Groups.h').replace('#include "Input.h"',
        '#include "lib_src/solvers/cin_advance/Groups.h"\n#include "FrozenSummary.h"')
    groups = groups.replace('#include "../../constraints/NodalRigidGroupCandidate.h"', '')
    groups = groups.replace('#include "../NodalNodeStep.h"', '')
    groups = groups.replace('namespace tl::fea::cin_advance::groups {',
        'namespace tl::fea::cooperative_test::frozen_groups {\nusing namespace cin_advance;\nusing cin_advance::groups::Report;\nusing cin_advance::groups::Threads;')
    groups = groups.replace('cin_timestep::detail::', 'frozen::')
    capture = read('Capture.h').replace('#include "Witness.h"',
        '#include "lib_src/solvers/cin_limiter/Witness.h"')
    capture = capture.replace('#include "../cin_timestep/ScreenValues.h"', '#include "FrozenValues.h"')
    capture = capture.replace('namespace tl::fea::cin_limiter {',
        'namespace tl::fea::cooperative_test::frozen_limiter {\nusing cin_limiter::Witness;')
    capture = capture.replace('cin_timestep::detail::', 'frozen::')
    kernels = read('Screen.cu').replace('#include "Screen.h"', '#include "FrozenSummary.h"')
    kernels = kernels.replace('#include "Groups.h"', '#include "FrozenGroups.h"')
    kernels = kernels.replace('#include "../cin_limiter/Capture.h"', '#include "FrozenCapture.h"')
    kernels = kernels.replace('namespace tl::fea::cin_advance::screen {',
        'namespace tl::fea::cooperative_test::frozen_screen {\nusing cin_advance::NoFailure;')
    kernels = kernels.replace('cin_timestep::detail::', 'frozen::')
    kernels = kernels.replace('groups::', 'frozen_groups::')
    kernels = kernels.replace('cin_limiter::Capture', 'frozen_limiter::Capture')
    for name in ('Merge','Observe','Complete'):
        kernels = kernels.replace(name+'(', 'frozen_screen::'+name+'(')
    return {'FrozenRigid.h':rigid, 'FrozenValues.h':values, 'FrozenScreen.h':screen,
        'FrozenSummary.h':summary, 'FrozenGroups.h':groups, 'FrozenCapture.h':capture,
        'FrozenScreen.cu':kernels}
if __name__ == '__main__':
    for name,data in generated().items():
        if '--check' in sys.argv:
            assert (HERE/name).read_text() == data, name
        else:
            (HERE/name).write_text(data)
