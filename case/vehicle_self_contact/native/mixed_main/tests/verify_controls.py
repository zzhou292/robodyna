#!/usr/bin/env python3
from pathlib import Path
import argparse,json,hashlib,subprocess,sys
p=argparse.ArgumentParser();p.add_argument('--workspace',type=Path,required=True);a=p.parse_args()
r=Path(__file__).resolve().parent;m=json.loads((r/'source-controls.json').read_text())
def read(row,blob=False):
    b=(a.workspace/row['path']).read_bytes()
    assert len(b)==row['bytes'] and hashlib.sha256(b).hexdigest()==row['sha256']
    if blob:assert hashlib.sha1(b'blob '+str(len(b)).encode()+bytes([0])+b).hexdigest()==row['git_blob']
    return b.decode()
audit=json.loads(read(m['gap_audit']));assert audit['revision']==m['revision']
texts={Path(row['path']).name:read(row,True) for row in audit['source_pins']}
c=texts['convertcontacts.cxx'];rd=texts['hm_read_inter_type25.F'];defaults=texts['definter.F'];control=texts['contrl.F']
assert 'interTypeVsMapDefaultVals["TYPE25"] = { {"Idel", 1}, {"Inacti", 5}, {"IGAP", 2} };' in c
assert 'lsdSFST = (lsdSFST == 0.0) ? 1.0 : lsdSFST;' in c
assert 'lsdSFMT = (lsdSFMT == 0.0) ? 1.0 : lsdSFMT;' in c
assert 'masterGapMax = 0.0;' in c and 'slaveGapMax  = 0.0;' in c
for phrase in ['IF(IGAP==2) THEN','IPARI(21)=1','ILEV = 1','IPARI(53)=IGAP0','IPARI(91)=ITHK',
    'IF(GAPSCALE==ZERO)GAPSCALE=ONE','IF(GAPMAX_S==ZERO)GAPMAX_S=EP30','IF(GAPMAX_M==ZERO)GAPMAX_M=EP30']:
    assert phrase in rd,phrase
type25=defaults.split('ELSEIF(ITYP == 25) THEN',1)[1]
for key in ['IGAP0','ITHK','IEDGE']:
    at=type25.index("CASE ('"+key+"')");assert 'DEF_DEF = 1000' in type25[at:at+95]
assert 'IF(IVAL == 1000 .AND. IS_DEFAUT_1000 ) IVAL = 0' in defaults
assert "CALL HM_OPTION_COUNT('/INTTHICK/V5',IINTTHICK)" in control
# Complete pinned converter inventory selects the fresh destination/no native
# preload absence premise already enforced by the private import/source handles.
private=[texts[Path(row['path']).name] for row in audit['source_pins'] if '/dyna2rad/_private/' in row['path']]
assert len(private)==audit['converter_private_translation_units']==40
for text in private:assert '/INTTHICK/V5' not in text and '/DEFAULT/INTER' not in text
sort=read(m['roster_sort']);assert 'ITRI(N)  =ITAB(INTBUF_TAB%NSV(N))' in sort
assert 'CALL MY_ORDERS(0,WORK,ITRI,INDEX,NSN,1)' in sort
assert 'INTBUF_TAB%NSV(N)    =ISAV(1,INDEX(N))' in sort
assert 'MSR' not in sort
assert 'destCard = "/PROP/TYPE1";' in texts['convertprops.cxx']
assert 'if (IridHandle.IsValid())' in texts['convertprops.cxx']
assert 'IGEO(11)=IGTYP' in texts['hm_read_prop01.F']
controls=json.loads((r/'../../main_coefficients/tests/post-gapm-controls.json').read_text())
surfi=next(read(row,True) for row in controls['files'] if row['path'].endswith('/i25surfi.F'))
s1=surfi.split('c     Surface nodes S1: Build Tags, Set NSV, Msr If Iallo = 2',1)[1]
s1=s1.split('c     nodes of the nod1 nod group',1)[0]
for phrase in ['IF(TAGS(I) == 0 .AND. ILEV /= 3 ) THEN','NSV(NSN) = I',
    'IF(TAG(I) == 1 .or. TAG(I) == -3)THEN','IF(IALLO == 2)MSR(NMN) = I']:
    assert phrase in s1,phrase
subprocess.run([sys.executable,'-B',str(r/'../../main_coefficients/tests/verify_postgapm_controls.py'),'--workspace',str(a.workspace)],check=True)
print(json.dumps({'status':'passed','native_donors':len(texts),'profile':m['gap_profile'],'scope':m['scope']}))
