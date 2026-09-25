#!/usr/bin/env python3
"""Deterministic qualification fixture binding; not a source/model producer."""
from pathlib import Path
import argparse,json,hashlib
q=Path(__file__).resolve().parent
parser=argparse.ArgumentParser();parser.add_argument('--check',action='store_true');args=parser.parse_args()
manifest=json.loads((q/'observed-primary-manifest.json').read_text())
path=q/manifest['file'];data=path.read_bytes()
assert len(data)==manifest['bytes'] and hashlib.sha256(data).hexdigest()==manifest['sha256']
fixture=json.loads(data);assert fixture['observation_sha256']==manifest['origin_observation_sha256']
main=fixture['main'];chunks=fixture['chunks']
assert main['clock']['TT']==0
assert [item['controls']['ESHIFT'] for item in chunks]==[0,2,4,6]
assert all(item['clock']['TT']==0 and item['clock']['DT1']==0 and not item['removal_nodes'] for item in chunks)
assert len(main['arrays']['X'])==54 and len(main['arrays']['V'])==54
for item in chunks:
    domain=item['arrays']['XYZM']
    assert all(domain[axis]<=main['arrays']['X'][3*node+axis]<=domain[axis+3] for node in range(18) for axis in range(3))
def arr(kind,name,values):
 vals=','.join(float(v).hex() if kind=='double' else str(v) for v in values)
 return f'  const {kind} {name}[]={{'+vals+'};\n'
s='''#pragma once
#include "PackingFixture.h"
#include "PrimaryContextFixture.h"
// Immutable qualification fixture from observation2; no production data source.
namespace candidate_test {
struct ObservedPrimaryRow {c::LocalRow row;int worker_count=0,unshifted_role=0;};
inline std::vector<ObservedPrimaryRow> ObservedPrimaryRows() {
'''
s+=arr('double','xyz',main['arrays']['X'])+arr('double','velocity',main['arrays']['V'])+arr('int','code',main['arrays']['ICODT'])
s+=arr('int','secondary',chunks[0]['arrays']['NSV'])+arr('double','secondary_gap',chunks[0]['arrays']['GAP_S'])
s+=arr('int','roles',chunks[0]['arrays']['MSEGTYP'])
s+='  std::vector<ObservedPrimaryRow> result;\n'
for chunk in chunks:
 c=chunk['controls'];a=chunk['arrays'];v=chunk['scalars'];shift=c['ESHIFT'];n=c['NRTM']
 s+='  {\n'+arr('int','nodes',a['IRECT'])+arr('double','main_gap',a['GAP_M'])+arr('double','curvature',a['CURV_MAX'])
 s+=f'''    for(unsigned m=0;m<{n};++m)for(unsigned secondary_row=0;secondary_row<18;++secondary_row) {{
      c::LocalRow row;const auto slave=secondary[secondary_row];bool own=false;
      for(unsigned j=0;j<4;++j)if(nodes[4*m+j]==slave)own=true;
      if(own)continue; // Native TRIVOX source exclusion precedes the packet.
      row.secondary_node=slave;row.main_count=8;row.segment_type=roles[{shift}+m];
      row.constraint_codes[4]=code[slave-1];row.previous_dt=0.;
      row.screen.secondary={{xyz[3*(slave-1)],xyz[3*(slave-1)+1],xyz[3*(slave-1)+2]}};
      row.secondary_velocity={{velocity[3*(slave-1)],velocity[3*(slave-1)+1],velocity[3*(slave-1)+2]}};
      row.screen.secondary_gap=secondary_gap[secondary_row];row.screen.main_gap=main_gap[m];
      row.screen.curvature=curvature[m];row.screen.margin={float(v['MARGE']).hex()};
      row.screen.drad={float(v['DRAD']).hex()};row.screen.gap_load={float(v['DGAPLOAD']).hex()};
      row.screen.stored_motion={float(v['VMAXDT']).hex()};
      for(unsigned j=0;j<4;++j){{const auto node=nodes[4*m+j];row.nodes[j]=node;row.constraint_codes[j]=code[node-1];
        row.screen.vertices[j]={{xyz[3*(node-1)],xyz[3*(node-1)+1],xyz[3*(node-1)+2]}};
        row.main_velocities[j]={{velocity[3*(node-1)],velocity[3*(node-1)+1],velocity[3*(node-1)+2]}};}}
      result.push_back({{row,{n},roles[m]}});
    }}
  }}
'''
s+='  return result;\n}\n} // namespace candidate_test\n';target=q/'ObservedPrimaryContext.h'
if args.check:
    assert target.read_text()==s, 'Observed header differs from the pinned structured fixture'
else:
    target.write_text(s)
print('Pinned observed-primary structured data and deterministic header: PASS')
