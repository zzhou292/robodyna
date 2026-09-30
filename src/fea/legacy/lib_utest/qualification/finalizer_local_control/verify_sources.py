#!/usr/bin/env python3
"""Exact reviewed destination-plumbing reversal; no compiler or GPU execution."""
from pathlib import Path
import hashlib,json
HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[2]
PINS={'baseline.json': 'e39211209ba38e39e1674175950f1a4594ef9e991f2465dedd7c9c7d2771d628', 'transformations.json': '578a26c175d59e4d335c6fb2561e0c25888d47952bedaa4c1e28f49fd51a8524', 'source-manifest.json': '11afc73adab850009507d184381c1e8e36dc00a55f7024a876e2b5096ad04dd8'}
def read(name):
    raw=(HERE/name).read_bytes();assert hashlib.sha256(raw).hexdigest()==PINS[name],name
    return json.loads(raw)
def verify():
    baseline=read('baseline.json');assert baseline['commit']=='2733b4ccb916db96fc4d7d51cdfa21566d6e227d'
    transformations={v['path']:v['changes'] for v in read('transformations.json')}
    for row in baseline['files']:
        frozen=(HERE/row['fixture']).read_bytes()
        assert len(frozen)==row['bytes'] and hashlib.sha256(frozen).hexdigest()==row['sha256']
        current=(ROOT/row['path']).read_text()
        for change in transformations[row['path']]:
            assert current.count(change['after'])==1,row['path']
            current=current.replace(change['after'],change['before'])
        assert current.encode()==frozen,row['path']
    manifest=read('source-manifest.json')
    for row in manifest['unchanged']:
        raw=(ROOT/row['path']).read_bytes()
        assert len(raw)==row['bytes'] and hashlib.sha256(raw).hexdigest()==row['sha256'],row['path']
    return {'status':'passed','complete2733_reversals':4,'unchanged_storage_numeric_owner_fixture_files':len(manifest['unchanged']),
        'source_only':True,'numerical_and_compiled_resource_qualification':False}
if __name__=='__main__':print(json.dumps(verify(),sort_keys=True))
