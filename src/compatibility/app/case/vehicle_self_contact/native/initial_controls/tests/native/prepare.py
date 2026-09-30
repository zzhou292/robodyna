#!/usr/bin/env python3
# Original selected-shell I25GAPM gap blocks and I25STI3 scalar sum only.
from pathlib import Path
import argparse,hashlib,json
p=argparse.ArgumentParser();p.add_argument('--tl-root',type=Path,required=True)
p.add_argument('--output',type=Path,required=True);p.add_argument('--check',action='store_true');a=p.parse_args()
donor=a.tl_root/'lib_utest/qualification/radioss_type25_coefficients/native/original/i25sti3.F'
data=donor.read_bytes();pin=json.loads((Path(__file__).parent/'donor.json').read_text())
assert len(data)==pin['bytes'] and hashlib.sha256(data).hexdigest()==pin['sha256']
s=data.decode();body=s[s.index('      SUBROUTINE I25GAPM'):]
def gap(part):
 start=part.index('          IF ( IGAP == 5.AND.THK_M /= ZERO) THEN')
 end=part.index('          GAP_M(I)=MAX(GAP_M(I),GAPM)',start)+len('          GAP_M(I)=MAX(GAP_M(I),GAPM)')
 return part[start:end]
tri=gap(body[body.index('        IF(NELTG/=0) THEN'):])
quad=gap(body[body.index('        ELSEIF(NELC/=0) THEN'):])
assert 'GAPS2=MAX(GAPS2,GAPM)' in tri and 'GAPS2=MAX(GAPS2,GAPM)' in quad
assert '        GAP = GAPS_MX+GAPM_MX' in s
source=r'''      SUBROUTINE RD_INITIAL_GAP(N,KIND,V,SECOND,CAP,R)
     . BIND(C,NAME="rd_initial_gap")
      USE ISO_C_BINDING
      IMPLICIT NONE
      INTEGER(C_INT) N,KIND(N),I,IP,MG,NELTG,NELC,NUMELC
      INTEGER IGAP,IINTTHICK,IGTYP,NDX
      REAL(C_DOUBLE) V(3,N),SECOND,CAP,R(2),THK(1),THK_PART(1)
      REAL(C_DOUBLE) GEO(1,1),GAP_M(N),GAPM,GAPS2,GAPMN,DXM,DX
      REAL(C_DOUBLE) GAPSCALE,THK_M,THK_M_SCALE,GAPS_MX,GAPM_MX
      REAL(C_DOUBLE) GAP,ZERO,HALF
      PARAMETER (ZERO=0D0,HALF=.5D0)
      IP=1
      MG=1
      NELTG=1
      NELC=1
      NUMELC=0
      IGAP=1
      IINTTHICK=0
      IGTYP=1
      NDX=0
      GAPSCALE=1D0
      THK_M=0D0
      THK_M_SCALE=1D0
      GAPS2=ZERO
      GAPMN=HUGE(1D0)
      DXM=ZERO
      GAP_M=ZERO
      DO I=1,N
        THK_PART(1)=V(1,I)
        THK(1)=V(2,I)
        GEO(1,1)=V(3,I)
        IF(KIND(I)==3)THEN
''' +tri+ "\n        ELSEIF(KIND(I)==4)THEN\n"+quad+"\n        ENDIF\n      ENDDO\n"
source+=r'''      GAP_M=MIN(GAP_M,CAP)
      GAPS_MX=SECOND
      GAPM_MX=GAPS2
        GAP = GAPS_MX+GAPM_MX
      R(1)=GAPS2
      R(2)=GAP
      END
'''
a.output.mkdir(parents=True,exist_ok=True);out=a.output/'Gap.F'
if a.check:assert out.read_text()==source
else:out.write_text(source)
print('Original selected-shell gap reduction and earlier global scalar sum verified')
