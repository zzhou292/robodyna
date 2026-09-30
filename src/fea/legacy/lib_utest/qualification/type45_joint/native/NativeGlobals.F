      SUBROUTINE T45_RESET_CONTEXT(TIME,STEP,CYCLE,UNIT)
#include "implicit_f.inc"
#include "vect01_c.inc"
#include "com04_c.inc"
#include "com08_c.inc"
#include "com01_c.inc"
#include "scr05_c.inc"
#include "scr02_c.inc"
#include "sms_c.inc"
#include "scr17_c.inc"
      DOUBLE PRECISION TIME,STEP
      INTEGER CYCLE,UNIT
      LFT=1
      NFT=0
      IERR=0
      NUMELR=1
      NRBODY=2
      TT=TIME
      DT1=STEP
      DT12=STEP
      NCYCLE=CYCLE
      IRESP=2
      IDTMINS=0
      NODADT=0
      DTMINS=0
      DTFACS=1
      IOUT=UNIT
      END
