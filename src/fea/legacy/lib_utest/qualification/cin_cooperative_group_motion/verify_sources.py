#!/usr/bin/env python3
"""Exact source provenance only; no numerical or CUDA execution claim."""
from pathlib import Path
import hashlib,json
here=Path(__file__).resolve().parent;root=here.parents[2]
raw=(here/'source-manifest.json').read_bytes()
assert hashlib.sha256(raw).hexdigest()=='8ceae1f0a22e8b997052d090d0747de14552f2c35ebfe1c760a61839611b6786'
manifest=json.loads(raw)
def same(data,row):
 assert len(data)==row['bytes'] and hashlib.sha256(data).hexdigest()==row['sha256'],row['path']
for row in manifest['unchanged']:same((root/row['path']).read_bytes(),row)
old="""  for (std::uint32_t group = blockIdx.x*blockDim.x+threadIdx.x;
       group < input.groups.group_count; group += gridDim.x*blockDim.x) {
    input.group_reports[group] = AdvanceGroup(input, group);
  }"""
new="""  __shared__ group_motion::Tile tile;
  const auto group=blockIdx.x;
  if (input.capture.node) group_motion::Advance<true>(input,group,tile);
  else group_motion::Advance<false>(input,group,tile);"""
pairs=[('#include "Groups.h"\n#include "group_motion/Motion.cuh"','#include "Groups.h"'),(new,old),
 ('  Advance<<<input.groups.group_count, group_motion::Threads, 0, stream>>>(input);',
  '  const auto blocks = 1+(input.groups.group_count-1)/Threads;\n  Advance<<<blocks, Threads, 0, stream>>>(input);')]
row=manifest['dispatch_registration_baseline'][0];text=(root/row['path']).read_text()
for current,prior in pairs:
 assert text.count(current)==1; text=text.replace(current,prior)
same(text.encode(),row)
row=manifest['dispatch_registration_baseline'][1];text=(root/row['path']).read_text()
headers=['Storage.h','Primary.h','Wrench.h','Members.h','Orientation.h','Motion.cuh']
registration=', '.join('"cin_advance/group_motion/'+name+'"' for name in headers)+', '
assert text.count(registration)==1;same(text.replace(registration,'').encode(),row)
for name in headers:
 text=(root/'lib_src/solvers/cin_advance/group_motion'/name).read_text()
 assert 'cudaMalloc' not in text and 'atomicAdd' not in text and 'reinterpret_cast' not in text
print(json.dumps({'status':'source_passed','unchanged_records':len(manifest['unchanged']),
 'baseline_commit':manifest['baseline_commit'],'shared_tile_bytes':4368,
 'compiled':False,'numerical_execution':False,'gpu_execution':False}))
