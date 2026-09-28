"""Exact narrow current-domain reversal; all arithmetic and startup guards stay pinned."""
from pathlib import Path
import hashlib,json
HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[2]
MANIFEST_SHA256='8c30b3eb1f232b28a949c4ff8d97e49f648e9802e79483abec84cf99f3e9d762'
def digest(value):return hashlib.sha256(value).hexdigest()
def verify():
    raw=(HERE/'source-proof.json').read_bytes()
    if digest(raw)!=MANIFEST_SHA256:raise ValueError('Unreviewed current-domain proof')
    proof=json.loads(raw)
    for item in proof['changes']:
        before=(HERE/item['baseline_file']).read_bytes()
        current=(ROOT/item['path']).read_bytes()
        if digest(before)!=item['before_sha256'] or digest(current)!=item['after_sha256']:
            raise ValueError('Current-domain header identity changed: '+item['path'])
        text=current.decode()
        if text.count(item['after'])!=1 or text.replace(item['after'],item['before'],1).encode()!=before:
            raise ValueError('Unexpected current-domain arithmetic/guard change: '+item['path'])
    frozen=(HERE/proof['frozen_comparator']['path']).read_bytes()
    if digest(frozen)!=proof['frozen_comparator']['sha256']:raise ValueError('Frozen comparator changed')
    baseline=(HERE/'baseline/QephCurrentFrame.h').read_text()
    start=baseline.index('TL_QEPH_HD inline Status CurrentFrame(')
    end=baseline.index('\n}\n',start)+3
    expected=baseline[start:end].replace(' CurrentFrame(', ' FrozenCurrentFrame(',1)
    if frozen.decode().count(expected)!=1:raise ValueError('Frozen comparator is not exact baseline body')
    for path,sha in proof['unchanged'].items():
        if digest((ROOT/path).read_bytes())!=sha:raise ValueError('Protected QEPH source changed: '+path)
    return {'status':'passed','reversed_headers':len(proof['changes']),'unchanged_files':len(proof['unchanged'])}
if __name__=='__main__':print(json.dumps(verify(),sort_keys=True))
