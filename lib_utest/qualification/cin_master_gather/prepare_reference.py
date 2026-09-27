#!/usr/bin/env python3
"""Generate a namespace-only complete pre-gather CIN caller for qualification."""
from pathlib import Path
import argparse


def render(source):
    signature = 'cudaError_t FENodalState::Impl::LaunchCinAdvance'
    assert source.count(signature) == 1
    assert source.count('namespace tl::fea {') == 1
    assert source.count('cudaError_t cin_advance::Launch(') == 1
    body = source[:source.index(signature)]
    body = body.replace('namespace tl::fea {', 'namespace tl::fea::cin_gather_test {\nusing namespace cin_advance;', 1)
    body = body.replace('cudaError_t cin_advance::Launch(', 'cudaError_t LaunchFrozen(', 1)
    return body + '} // namespace tl::fea::cin_gather_test\n'


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('output')
    args = parser.parse_args()
    source = Path(__file__).with_name('reference') / 'ExplicitNodalCinStep.cu'
    Path(args.output).write_text(render(source.read_text()))
