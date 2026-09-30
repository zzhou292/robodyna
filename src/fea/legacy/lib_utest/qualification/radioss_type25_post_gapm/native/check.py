#!/usr/bin/env python3
"""Read-only pins and complete original donor reuse; no numerical execution."""
import hashlib,json,runpy,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parent
TL=ROOT.parents[3]
def verify():
    m=json.loads((ROOT/'source-manifest.json').read_text())
    parent=TL/m['parent_native']
    assert hashlib.sha256((parent/'source-manifest.json').read_bytes()).hexdigest()==m['parent_manifest_sha256']
    for row in m['source_files']:
        data=(ROOT/row['path']).read_bytes()
        assert len(data)==row['bytes'] and hashlib.sha256(data).hexdigest()==row['sha256'],row['path']
    sys.path.insert(0,str(parent))
    generated=runpy.run_path(str(parent/'prepare.py'))['generated']()
    for name in m['parent_reuse']:assert name in generated,name
    neighborhood=generated['Neighborhood.F']
    for token in ('MNEIGH_SOLID','ITAG_S','IF(IDEL_SOLID > 0)','CALL I25NEIGH_SEG_E'):
        assert token in neighborhood,token
    assert 'IELEM_M(2' in generated['StarterNormals.F']
    assert 'MVSIZ=129' in generated['engine_mvsiz_p.inc'].replace(' ','')
    return {'status':'source_verified_not_numerically_executed','revision':m['revision'],'reuse':m['parent_reuse']}
if __name__=='__main__':print(json.dumps(verify(),sort_keys=True))
