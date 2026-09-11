          DD = ZERO
          NODE1=IRECT(1,CAND_E(I))
          NODE2=IRECT(2,CAND_E(I))
          NODE3=IRECT(3,CAND_E(I))
          NODE4=IRECT(4,CAND_E(I))
C         LONGUEUR DIAG 1
          DX1=(X(1,NODE1)-X(1,NODE3))
          DY1=(X(2,NODE1)-X(2,NODE3))
          DZ1=(X(3,NODE1)-X(3,NODE3))
          DD=SQRT(DX1**2+DY1**2+DZ1**2)
C         LONGUEUR DIAG 2
          DX3=(X(1,NODE2)-X(1,NODE4))
          DY3=(X(2,NODE2)-X(2,NODE4))
          DZ3=(X(3,NODE2)-X(3,NODE4))
          DD=MIN(DD,SQRT(DX3**2+DY3**2+DZ3**2))
          GAPV(I) = MAX(ZEP05*DD,ZEP6*(THKSECND+THKMAIN))
