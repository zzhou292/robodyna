#!/usr/bin/env python3
from pathlib import Path
import argparse,hashlib,json
p=argparse.ArgumentParser();p.add_argument('--workspace',type=Path,required=True);a=p.parse_args()
r=Path(__file__).resolve().parent
manifest=json.loads((r/'post-gapm-controls.json').read_text())
assert manifest['revision']=='a62b27e6baa555d222a580d6218867d0be4d70b5'
texts=[]
for row in manifest['files']:
 b=(a.workspace/row['path']).read_bytes()
 assert len(b)==row['bytes'] and hashlib.sha256(b).hexdigest()==row['sha256']
 assert hashlib.sha1(b'blob '+str(len(b)).encode()+bytes([0])+b).hexdigest()==row['git_blob']
 texts.append(b.decode())
converter,cfg,reader,defaults,surfi,gapm=texts
assert 'interTypeVsMapDefaultVals["TYPE25"] = { {"Idel", 1}' in converter
assert 'for (auto tempPair : interTypeVsMapDefaultVals[interType])' in converter
assert 'radInterEdit.SetValue(sdiIdentifier(tempPair.first), sdiValue(tempPair.second));' in converter
assert 'RADIO(TYPE24_Idel, "Idel")' in cfg
assert "CALL HM_GET_INTV('TYPE24_Idel',IDEL25" in reader
assert 'IASSIGN = 1' in reader and 'IPARI(17)=IDEL25' in reader
assert 'ELSEIF(FLAG == 1 .AND. IPRINT == 0)THEN' in defaults
assert 'IF(IVAL == 0 .AND. DEF_INTER(INDEX) /= 0)THEN' in defaults
assert 'IF(IDEL > 0.AND.SOLID_SEGMENT>0) THEN' in surfi
assert 'IPARI(100) = 1' in surfi
assert 'IF(NSOL_INT == 0) THEN' in gapm and 'IDEL_SOLID = 0' in gapm
assert gapm.index('NSOL_INT = NSOL_INT + 1')<gapm.index('CALL INCOQ3')
print(json.dumps({'status':'passed','scope':manifest['scope'],'native_files':len(texts)}))
