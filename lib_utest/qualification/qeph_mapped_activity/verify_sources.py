#!/usr/bin/env python3
"""Verify complete frozen host validation and every owning/reused source."""
from pathlib import Path
import hashlib
import json
import re

here = Path(__file__).resolve().parent
root = here.parents[2]
manifest_bytes = (here / 'source-manifest.json').read_bytes()
assert hashlib.sha256(manifest_bytes).hexdigest() == 'e3367f0d6f4f99dd8c3c46fc5eb6c598eafa297a31053fca8e14eb2981976002'
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
mixed_reader = (here / 'FrozenMixedReadback.cpp.txt').read_text()
current_mixed_reader = (root / 'lib_src/elements/ShellMixedSectionReadback.cpp').read_text()
assert current_mixed_reader.startswith(mixed_reader)
expected_mixed_reader = mixed_reader.replace('#include "ShellMixedSectionStorage.h"',
    '#include "MixedOracleSupport.h"').replace('#include "ShellLayeredSectionValues.h"',
    '#include "SerialMixedValues.h"').replace('namespace tl::fea::shell_batch_plasticity_detail {',
    'namespace qeph_activity_test::frozen_mixed {')
expected_mixed_reader = '#pragma once\n' + expected_mixed_reader.replace(
    'SetupReport MixedHostStorage::Read','inline SetupReport MixedHostStorage::Read')
assert expected_mixed_reader == (here / 'SerialMixedReadback.h').read_text()
mixed_values = (here / 'FrozenMixedValues.h.txt').read_text()
expected_mixed_values = mixed_values.replace('#include "ShellBatchLayeredSection.h"',
    '#include "lib_src/elements/ShellBatchLayeredSection.h"').replace(
    'namespace tl::fea::shell_batch_plasticity_detail {',
    'namespace qeph_activity_test::frozen_mixed {\nusing namespace tl::fea;')
assert expected_mixed_values == (here / 'SerialMixedValues.h').read_text()
portable = mixed_values.replace('#include "ShellBatchLayeredSection.h"',
    '#include "ShellBatchLayeredSection.h"\n\n#if defined(__CUDACC__)\n'
    '#define TL_MIXED_VALUES_HD __host__ __device__\n#else\n#define TL_MIXED_VALUES_HD\n#endif')
portable = portable.replace('inline bool FiniteSection','TL_MIXED_VALUES_HD inline bool FiniteSection')
portable = portable.replace('  for(double x:{d.plastic_work_density_increment',
    '  const double values[]{d.plastic_work_density_increment').replace(
    '      value.cumulative_plastic_work_J})if(!tl::math::Finite(x))return false;',
    '      value.cumulative_plastic_work_J};\n  for(double x:values)if(!tl::math::Finite(x))return false;')
assert portable + '\n#undef TL_MIXED_VALUES_HD\n' == (root / 'lib_src/elements/ShellLayeredSectionValues.h').read_text()
print(json.dumps({'status':'passed','records':len(manifest['files']),
    'complete_frozen_functions':2,'baseline':manifest['baseline_commit'],
    'complete_mixed_reader_and_mechanical_values':True,
    'complete_failure_values_and_full_read_unchanged':True,'numerical_execution':False}))
