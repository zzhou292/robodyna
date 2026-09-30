#!/usr/bin/env python3
"""Exact qualification-only namespace/state decoration of complete read bodies."""
from pathlib import Path
import re

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]


def body(source, declaration):
    start = source.index(declaration)
    first = source.index('{', start)
    level = 1
    end = first + 1
    while level:
        level += (source[end] == '{') - (source[end] == '}')
        end += 1
    return source[first:end]


def qualify(value):
    value = value.replace('*this', 'state')
    for name in ('plasticity', 'joined_binding', 'physical', 'config', 'storage',
                 'staging', 'accepted_stamp', 'PendingError', 'Runtime',
                 'AcceptedSlabIndex', 'ReadResults', 'ValidateMappedResults',
                 'ValidateMappedSections', 'ValidateOnePointReadback'):
        value = re.sub(r'(?<![.\w])' + name + r'\b', 'state.' + name, value)
    return value


def generated():
    old = (HERE / 'FrozenOnePoint.cpp.txt').read_text()
    mapped = (HERE / 'FrozenMapped.cpp.txt').read_text()
    failure = (HERE / 'FrozenFailure.cpp.txt').read_text()
    current = (ROOT / 'lib_src/elements/t3/T3BatchOnePointReadback.cpp').read_text()
    prefix = ('// Generated from complete authenticated bodies; no numerical edits.\n'
              '#pragma once\n#include "lib_src/elements/t3/T3Batch.h"\n#include "lib_src/elements/t3/T3OnePointHistory.h"\n'
              '#include "lib_src/elements/t3/mapped/Result.h"\n'
              '#include "lib_src/elements/failure/ShellFailureReadback.h"\n')
    using = ('using namespace tl::fea;\nusing namespace tl::fea::t3;\n')
    serial = prefix + 'namespace t3_readback_test::serial {\n' + using
    serial += ('template<class State> BatchReport OnePoint(State& state, unsigned slab, double time, '
               'std::uint64_t epoch) ' + qualify(body(old,
               'BatchReport T3Batch::Impl::ValidateOnePointReadback')) + '\n')
    for name in ('ValidateMappedResults', 'ValidateMappedSections'):
        serial += ('template<class State> BatchReport ' + name +
                   '(State& state, unsigned slab) ' + qualify(body(mapped,
                   'BatchReport T3Batch::Impl::' + name)) + '\n')
    first = failure.index('  const auto report = shell_batch_plasticity_detail::ReadFailure(state,',
                          failure.index('BatchReport T3Batch::CopyAcceptedParentActivity'))
    last = failure.index('  const auto* history =', first)
    activity = failure[first:last].replace('state.AcceptedSlabIndex()', 'slab').replace(
        'state.accepted_diagnostics.time', 'time').replace('state.accepted_stamp.epoch', 'epoch')
    serial += ('template<class State> BatchReport Activity(State& state, unsigned slab, double time, '
               'std::uint64_t epoch) {\n' + activity + '  return report;\n}\n}\n')
    current_helper = current[current.index('template<class State, class ReadResults>'):
                             current.index('} // namespace\n')]
    candidate = prefix + 'namespace t3_readback_test::candidate {\n' + using + current_helper
    candidate += ('template<class State> BatchReport Activity(State& state, unsigned slab, double time, '
                  'std::uint64_t epoch) ' + qualify(body(current,
                  'BatchReport T3Batch::Impl::ReadParentActivity')) + '\n}\n')
    # The extracted helper differs only by explicit state and the transport seam.
    expected = qualify(body(old, 'BatchReport T3Batch::Impl::ValidateOnePointReadback')).replace(
        'state.ReadResults(&state.storage->slab[slab])', 'read_results()')
    assert body(current_helper, 'BatchReport ReadOnePoint') == expected
    return {'SerialFlow.h': serial, 'CandidateFlow.h': candidate}


if __name__ == '__main__':
    for name, text in generated().items():
        (HERE / name).write_text(text)
