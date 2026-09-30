#!/usr/bin/env python3
"""Read-only current wrapper and complete reused NORMP donor check; no compiler."""
import hashlib,json,sys,runpy
from pathlib import Path
ROOT=Path(__file__).resolve().parent
TL=ROOT.parents[3]
def verify():
    m=json.loads((ROOT/'source-manifest.json').read_text())
    parent=TL/m['parent_native']
    assert hashlib.sha256((parent/'source-manifest.json').read_bytes()).hexdigest()==m['parent_manifest_sha256']
    for entry in m['source_files']:
        b=(ROOT/entry['path']).read_bytes()
        assert len(b)==entry['bytes'] and hashlib.sha256(b).hexdigest()==entry['sha256']
    sys.path.insert(0,str(parent))
    namespace=runpy.run_path(str(parent/'prepare.py'))
    generated=namespace['generated']()
    assert all(name in generated for name in m['parent_reuse'])
    assert 'SUBROUTINE I25NORMP' in generated['ReadyNormals.F']
    assert 'SUBROUTINE I25FREE_BOUND' in generated['FreeMain.F']
    assert 'MVSIZ=129' in generated['engine_mvsiz_p.inc'].replace(' ','')
    return {'status':'source_verified_not_numerically_executed','donor_revision':m['donor_revision'],
            'reuse':m['parent_reuse'],'controls':m['runtime_controls']}
if __name__=='__main__':print(json.dumps(verify(),sort_keys=True))
