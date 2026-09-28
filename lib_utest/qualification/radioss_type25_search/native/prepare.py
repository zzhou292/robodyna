#!/usr/bin/env python3
"""Generate test-only native boundaries without translating numerical bodies."""
import argparse,hashlib,json,re
from pathlib import Path
here=Path(__file__).resolve().parent

def generated():
 for entry in json.loads((here/'source-manifest.json').read_text())['files']:
  raw=(here/entry['path']).read_bytes()
  assert len(raw)==entry['bytes'] and hashlib.sha256(raw).hexdigest()==entry['sha256'],entry['path']
 constants=(here/'original/constant_mod.F').read_text()
 names={'ZERO','ONE','TWO','FIVE','TEN','HUNDRED','HALF','EP30','EM02','ZEP01','ONEP01'}
 names.update('EP%02d'%i for i in range(2,21))
 lines=[]
 for line in constants.splitlines():
  match=re.search(r'my_real, parameter ::\s*(\w+)\s*=',line)
  if match and match[1] in names:lines.append(line.replace('my_real','real(c_double)',1))
 assert len(lines)==len(names)
 module='module search_constants\n use iso_c_binding\n implicit none\n'+'\n'.join(lines)+'\nend module\n'
 module+='module search_precision\n use iso_c_binding\n implicit none\n integer,parameter :: wp=c_double\nend module\n'
 xsave=(here/'original/i25xsave.F90').read_text()
 # Only module/dependency names change; the complete original numerical body remains.
 xsave=re.sub(r'(?i)\bmodule i25xsave_mod\b','module search_xsave_native',xsave)
 xsave=re.sub(r'(?i)\bUSE PRECISION_MOD\b','USE search_precision',xsave)
 xsave=re.sub(r'(?i)\bUSE CONSTANT_MOD\b','USE search_constants',xsave)
 ext=(here/'original/i25buce_crit.F').read_text()
 ext=ext[ext.index('       SUBROUTINE I25BUCE_CRIT('):]
 budget=(here/'original/intcrit.F').read_text()
 marker='        ELSEIF(NTY == 25)THEN'
 budget=budget.split(marker,1)[1].split('        ELSE ! all other NTYP',1)[0]
 gap=(here/'original/i25gap3.F').read_text()
 start='      DO I=NRTMF,NRTML\n        MAXDGAP_L = MAX(MAXDGAP_L,GAP_M(I)-GAPMSAV(I))'
 gap=start+gap.split(start,1)[1].split('      END DO',1)[0]+'      END DO\n'
 return {'Constants.F90':module,'Boundary.F90':(here/'Boundary.F90').read_text(),
  'Reference.F90':(here/'Reference.F90').read_text(),'Xsave.F90':xsave,
  'Extrema.F':ext,'Budget.F':(here/'Budget.F.in').read_text().replace('@BODY@',budget),
  'Gap.F':(here/'Gap.F.in').read_text().replace('@BODY@',gap),
  'implicit_f.inc':'      USE ISO_C_BINDING\n      USE SEARCH_CONSTANTS\n      IMPLICIT NONE\n#define my_real REAL(C_DOUBLE)\n',
  'com01_c.inc':'      INTEGER NUMNOD,NTHREAD,NSPMD\n      COMMON /SEARCH_DOMAIN/ NUMNOD,NTHREAD,NSPMD\n',
  'com04_c.inc':'','task_c.inc':'','comlock.inc':'','lockon.inc':'','lockoff.inc':''}
if __name__=='__main__':
 parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path);parser.add_argument('--check',action='store_true');args=parser.parse_args()
 generated_files=generated()
 for name,text in generated_files.items():
  if args.output is None:continue
  path=args.output/name
  if args.check:assert path.read_text()==text,name
  else:path.parent.mkdir(parents=True,exist_ok=True);path.write_text(text)
 print('PASS: pinned whole XSAVE/extrema and NTY25 budget/gap source boundaries')
