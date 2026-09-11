        XMSI=MS(I)*WEIGHT(I)
        FS(1)=A(1,I)*WEIGHT(I)
        FS(2)=A(2,I)*WEIGHT(I)
        FS(3)=A(3,I)*WEIGHT(I)
        IF (IRODDL==1) THEN
          INS=IN(I)*WEIGHT(I)
          MX=AR(1,I)*WEIGHT(I) + YS*FS(3) - ZS*FS(2)
          MY=AR(2,I)*WEIGHT(I) + ZS*FS(1) - XS*FS(3)
          MZ=AR(3,I)*WEIGHT(I) + XS*FS(2) - YS*FS(1)
        ELSE
          INS=ZERO
          MX=YS*FS(3) - ZS*FS(2)
          MY=ZS*FS(1) - XS*FS(3)
          MZ=XS*FS(2) - YS*FS(1)
        ENDIF
C
        A1=DET*(MX*B1+MY*C3+MZ*C2)
        A2=DET*(MY*B2+MZ*C1+MX*C3)
        A3=DET*(MZ*B3+MX*C2+MY*C1)
C
        FX0=FS(1)*FOURTH
        FY0=FS(2)*FOURTH
        FZ0=FS(3)*FOURTH
C------------------------------------------------------
C     FORCES TRANSMISES AUX NODES MAINS
C------------------------------------------------------
        FX(1) = FX0 + A2*Z1 - A3*Y1
        FY(1) = FY0 + A3*X1 - A1*Z1
        FZ(1) = FZ0 + A1*Y1 - A2*X1
        FX(2) = FX0 + A2*Z2 - A3*Y2
        FY(2) = FY0 + A3*X2 - A1*Z2
        FZ(2) = FZ0 + A1*Y2 - A2*X2
        FX(3) = FX0 + A2*Z3 - A3*Y3
        FY(3) = FY0 + A3*X3 - A1*Z3
        FZ(3) = FZ0 + A1*Y3 - A2*X3
        FX(4) = FX0 + A2*Z4 - A3*Y4
        FY(4) = FY0 + A3*X4 - A1*Z4
        FZ(4) = FZ0 + A1*Y4 - A2*X4
