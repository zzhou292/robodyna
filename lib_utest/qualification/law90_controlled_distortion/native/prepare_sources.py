"""Authenticate the LAW90 material/caller boundary and stage the exact PM107 assignment."""
import argparse,hashlib,json
from pathlib import Path
HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[3]
def prepare(output,check):
    manifest=json.loads((HERE/'source-manifest.json').read_text())
    assert manifest['revision']=='a62b27e6baa555d222a580d6218867d0be4d70b5'
    text={}
    for key,item in manifest['sources'].items():
        raw=(ROOT/item['path']).read_bytes()
        assert len(raw)==item['bytes'] and hashlib.sha256(raw).hexdigest()==item['sha256'],key
        text[key]=raw.decode('latin1')
    reader=text['reader'].lower()
    for value in ['bulk  = parmat(1)','pm(21,i) = nu','pm(32,i) = bulk',
                  'if (pm(100,i) == zero) pm(100,i) = pm(32,i)']:
        assert value in reader,value
    caller=text['caller']
    assert 'GBUF%SIG ,GBUF%RHO ,CXX        ,OFFG' in caller
    assert 'GBUF%OFF ,LL       ,VOLN       ,FLD' in caller
    assert 'STI,   STI_C,   FLD  ,    CNS2' in caller
    assert 'NC7,     NC8,     STIFN,   STIN' in caller
    lines=text['update'].splitlines(keepends=True)
    selected=[line for line in lines if 'PM(107,IMAT) = TWO*MAX(PM(32,IMAT),PM(100,IMAT))' in line]
    assert len(selected)==1
    raw=''.join(selected).encode('latin1');target=output/'pm107.inc'
    if check: assert target.read_bytes()==raw
    else: output.mkdir(parents=True,exist_ok=True);target.write_bytes(raw)
if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True);parser.add_argument('--check',action='store_true')
    args=parser.parse_args();prepare(args.output,args.check)
