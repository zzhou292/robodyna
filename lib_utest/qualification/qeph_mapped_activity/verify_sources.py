#!/usr/bin/env python3
"""Verify complete frozen host validation and every owning/reused source."""
from pathlib import Path
import hashlib
import json
import re

here = Path(__file__).resolve().parent
root = here.parents[2]
manifest_bytes = (here / 'source-manifest.json').read_bytes()
assert hashlib.sha256(manifest_bytes).hexdigest() == 'efc9dfdc210e743e3258b2134eae6ad539970990c245c3a344d8ca89b3ab3cfd'
manifest = json.loads(manifest_bytes)
for row in manifest['files']:
    path = Path(row['path'])
    assert not path.is_absolute() and '..' not in path.parts
    value = (root / path).read_bytes()
    assert len(value) == row['bytes'], path
    assert hashlib.sha256(value).hexdigest() == row['sha256'], path

source = (here / 'FrozenReadback.cpp.txt').read_text()
assert len(source.encode()) == manifest['baseline_bytes']
assert hashlib.sha256(source.encode()).hexdigest() == manifest['baseline_sha256']
first = source.index('BatchReport QephBatch::Impl::ValidateMappedResults')
second = source.index('BatchReport QephBatch::Impl::ValidateMappedSections')
end = source.index('} // namespace tl::fea::qeph')
results = source[first:second].replace(
    'BatchReport QephBatch::Impl::ValidateMappedResults(unsigned slab) const noexcept {',
    'template<class State>\nBatchReport ValidateMappedResults(const State& state,unsigned slab) noexcept {')
sections = source[second:end].replace(
    'BatchReport QephBatch::Impl::ValidateMappedSections(unsigned slab) {',
    'template<class State>\nBatchReport ValidateMappedSections(State& state,unsigned slab) {')
for name in ['physical','AcceptedSlabIndex','accepted_stamp','config','staging','ReadResults','storage','plasticity']:
    results = re.sub(r'(?<![.\w])' + name + r'\b','state.' + name,results)
    sections = re.sub(r'(?<![.\w])' + name + r'\b','state.' + name,sections)
expected = ('// Complete baseline validation bodies; only explicit State qualification changed.\n'
    '#pragma once\n#include "lib_src/elements/qeph/mapped/Result.h"\n'
    'namespace qeph_activity_test::serial {\nusing namespace tl::fea;\n'
    'using namespace tl::fea::qeph;\n' + results + sections + '}\n')
assert expected == (here / 'SerialValidation.h').read_text()
failure = (here / 'FrozenFailureValues.h.txt').read_text()
expected_failure = failure.replace('#include "../ShellBatchFailure.h"',
    '#include "lib_src/elements/ShellBatchFailure.h"').replace('#include "../ShellBatchPlasticity.h"',
    '#include "lib_src/elements/ShellBatchPlasticity.h"').replace(
    'namespace tl::fea::shell_batch_plasticity_detail {',
    'namespace qeph_activity_test::frozen_failure {\nusing namespace tl::fea;\nnamespace material = tl::material;')
assert expected_failure == (here / 'SerialFailureValues.h').read_text()
current_values = (root / 'lib_src/elements/failure/ShellFailureValues.h').read_text()
assert current_values.replace('TL_RESIDENT_FAILURE_HD inline bool ValidFailureEncoding',
    'inline bool ValidFailureEncoding') == failure
# Full Read retains its entire source/preflight/union/copy/validation body.
marker = 'SetupReport FailureHostStorage::Read(unsigned slab, std::size_t count,'
frozen_storage = (here / 'FrozenFailureStorage.cpp.txt').read_text()
current_storage = (root / 'lib_src/elements/failure/ShellFailureStorage.cpp').read_text()
assert frozen_storage[frozen_storage.index(marker):] == current_storage[current_storage.index(marker):]
print(json.dumps({'status':'passed','records':len(manifest['files']),
    'complete_frozen_functions':2,'baseline':manifest['baseline_commit'],
    'complete_failure_values_and_full_read_unchanged':True,'numerical_execution':False}))
