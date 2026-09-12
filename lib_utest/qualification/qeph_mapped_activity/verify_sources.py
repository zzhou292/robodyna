#!/usr/bin/env python3
"""Verify complete frozen host validation and every owning/reused source."""
from pathlib import Path
import hashlib
import json
import re

here = Path(__file__).resolve().parent
root = here.parents[2]
manifest_bytes = (here / 'source-manifest.json').read_bytes()
assert hashlib.sha256(manifest_bytes).hexdigest() == 'c05e879f73dd0d77d56ccb48b15be2e5079a2121b9479d83ace925f95868f5ce'
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
print(json.dumps({'status':'passed','records':len(manifest['files']),
    'complete_frozen_functions':2,'baseline':manifest['baseline_commit'],
    'numerical_execution':False}))
