#!/usr/bin/env python3
"""Only include/namespace adaptation of complete immutable 92e8cc4 donors."""
from pathlib import Path
import sys

HERE = Path(__file__).resolve().parent


def generated():
    force = (HERE/'reference/CinForceStage.h').read_text()
    force = force.replace('#include "CinStageTypes.h"',
                          '#include "lib_src/constraints/tied_shell/runtime/CinStageTypes.h"')
    force = force.replace('#include "../TiedPatchForce.h"',
                          '#include "lib_src/constraints/tied_shell/TiedPatchForce.h"')
    force = force.replace('namespace tl::constraints::tied_shell::cin {',
                          'namespace tl::constraints::tied_shell::cin_transfer_frozen {\nusing namespace cin;')
    force = force.replace('} // namespace tl::constraints::tied_shell::cin',
                          '} // namespace tl::constraints::tied_shell::cin_transfer_frozen')
    caller = (HERE/'reference/ExplicitNodalCinStep.cu').read_text()
    caller = caller[:caller.index('cudaError_t FENodalState::Impl::LaunchCinAdvance')]
    caller = '#include "FrozenForce.h"\n'+caller
    caller = caller.replace('namespace tl::fea {',
                            'namespace tl::fea::cin_transfer_test {\nusing namespace cin_advance;')
    caller = caller.replace('cudaError_t cin_advance::Launch(', 'cudaError_t LaunchFrozen(')
    caller = caller.replace('cin::detail::TransferForceTrial(model, force)',
                            'constraints::tied_shell::cin_transfer_frozen::detail::TransferForceTrial(model, force)')
    caller = caller.replace('cin::PrepareForceTrial(model, force)',
                            'constraints::tied_shell::cin_transfer_frozen::PrepareForceTrial(model, force)')
    caller += '} // namespace tl::fea::cin_transfer_test\n'
    return {'FrozenForce.h': force, 'Frozen.cu': caller}


if __name__ == '__main__':
    for name, contents in generated().items():
        if '--check' in sys.argv:
            assert (HERE/name).read_text() == contents, name
        else:
            (HERE/name).write_text(contents)
