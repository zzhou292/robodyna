"""Compile the unchanged consumed CHKMSR3NB TYPE25 prefix, never a production dependency."""
import argparse
import hashlib
import json
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('--source',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--check',action='store_true');args=p.parse_args()
manifest=json.loads((Path(__file__).parent/'source.json').read_text());data=(args.source/manifest['path']).read_bytes()
assert len(data)==manifest['bytes'] and hashlib.sha256(data).hexdigest()==manifest['sha256']
text=data.decode('ascii');start=text.index('      SUBROUTINE CHKMSR3NB(');begin=text.index('      NMNF = 1 + ITASK*NMN / NTHREAD',start)
last='      IF(NTY==7.OR.NTY==10.OR.NTY==22.OR.NTY==24.OR.NTY==25) RETURN\n';end=text.index(last,begin)+len(last);body=text[begin:end]
assert body.count('CALL MY_BARRIER()')==1
body=body.replace('CALL MY_BARRIER()','CALL MSR_TEST_BARRIER()')
header="""      SUBROUTINE ROBO_MSR_RETIREMENT(NMN,NUMNOD,ITAG,MSR)
     . BIND(C,NAME="robo_msr_retirement")
      USE ISO_C_BINDING
      IMPLICIT NONE
      INTEGER(C_INT),VALUE :: NMN,NUMNOD
      INTEGER(C_INT),INTENT(IN) :: ITAG(NUMNOD)
      INTEGER(C_INT),INTENT(INOUT) :: MSR(NMN)
      INTEGER I,NMNF,NMNL,ICOMP,ITASK,NTHREAD,NTY
      ITASK=0
      NTHREAD=1
      NTY=25
"""
footer="""      ERROR STOP 'Original TYPE25 early return was not taken'
      END SUBROUTINE
      SUBROUTINE MSR_TEST_BARRIER()
      IMPLICIT NONE
      RETURN
      END SUBROUTINE
"""
output=(header+body+footer).encode('ascii');path=args.output/'MainRole.F'
if args.check:
 assert path.read_bytes()==output
 print('Pinned CHKMSR3NB TYPE25 prefix and serial adapter verified')
else:
 args.output.mkdir(parents=True,exist_ok=True);path.write_bytes(output)
