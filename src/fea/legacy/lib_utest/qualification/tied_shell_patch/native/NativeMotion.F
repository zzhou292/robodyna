      SUBROUTINE I2VIROT3(NSN,NMN,A   ,IRECT,DPARA,MSR ,
     2                  NSV,IRTL,V   ,MS    ,AR  ,VR   ,
     3                  X  ,WEIGHT )
C-----------------------------------------------
C   I m p l i c i t   T y p e s
C-----------------------------------------------
#include      "implicit_f.inc"
C-----------------------------------------------
C   D u m m y   A r g u m e n t s
C-----------------------------------------------
      INTEGER NSN, NMN,
     .   IRECT(4,*), MSR(*), NSV(*), IRTL(*),WEIGHT(*)
C     REAL
      my_real
     .    A(3,*), DPARA(7,*), V(3,*),
     .    MS(*),VR(3,*),AR(3,*),X(3,*)
C-----------------------------------------------
C   C o m m o n   B l o c k s
C-----------------------------------------------
#include      "com01_c.inc"
C-----------------------------------------------
C   L o c a l   V a r i a b l e s
C-----------------------------------------------
      INTEGER NIR, I, J1, J2, J3, J4, II, L
C     REAL
      my_real
     . AMX, AMY, AMZ, VMX, VMY, VMZ,
     .   MRX,MRY,MRZ,MGX,MGY,MGZ,DET,ARX,ARY,ARZ,
     .   X0,X1,X2,X3,X4,XS,Y0,Y1,Y2,Y3,Y4,YS,Z0,Z1,Z2,Z3,Z4,ZS,
     .   VRX,VRY,VRZ,B1,B2,B3,C1,C2,C3
C-----------------------------------------------
      NIR=2
      IF(N2D==0)NIR=4
CFP (RESET MASSE MAIN)
C      DO II=1,NSN
C        I=NSV(II)
C        L=IRTL(II)
C        SS=CRST(1,II)
C        TT=CRST(2,II)
C        SP=1.0+SS
C        SM=1.0-SS
C        TP=.25*(1.0+TT)
C        TM=.25*(1.0-TT)
C        H(1)=TM*SM
C        H(2)=TM*SP
C        H(3)=TP*SP
C        H(4)=TP*SM
C        I1=3*I-2
C        XMSI=MS(I)*WEIGHT(I)
C        DO JJ=1,NIR
C         J=IRECT(JJ,L)
C         MS(J)=MS(J)-XMSI*H(JJ)
C        ENDDO
C      ENDDO
C------------------------------------
C     velocities of secondary nodes
C------------------------------------
      DO II=1,NSN
       I=NSV(II)
       IF(I>0)THEN
        L=IRTL(II)
C          J1=MSR(IRECT(1,L))
C          J2=MSR(IRECT(2,L))
C          J3=MSR(IRECT(3,L))
C          J4=MSR(IRECT(4,L))
        J1=IRECT(1,L)
        J2=IRECT(2,L)
        J3=IRECT(3,L)
        J4=IRECT(4,L)
C----------------------------------------
C       VELOCITY MOYENNE DU SEGMENT MAIN
C----------------------------------------
        VMX=FOURTH*(V(1,J1)+V(1,J2)+V(1,J3)+V(1,J4))
        VMY=FOURTH*(V(2,J1)+V(2,J2)+V(2,J3)+V(2,J4))
        VMZ=FOURTH*(V(3,J1)+V(3,J2)+V(3,J3)+V(3,J4))
        AMX=FOURTH*(A(1,J1)+A(1,J2)+A(1,J3)+A(1,J4))
        AMY=FOURTH*(A(2,J1)+A(2,J2)+A(2,J3)+A(2,J4))
        AMZ=FOURTH*(A(3,J1)+A(3,J2)+A(3,J3)+A(3,J4))
C----------------------------------------------------
C       average rotation velocity of the main segment
C----------------------------------------------------
        X1=X(1,J1)
        Y1=X(2,J1)
        Z1=X(3,J1)
        X2=X(1,J2)
        Y2=X(2,J2)
        Z2=X(3,J2)
        X3=X(1,J3)
        Y3=X(2,J3)
        Z3=X(3,J3)
        X4=X(1,J4)
        Y4=X(2,J4)
        Z4=X(3,J4)
        X0=FOURTH*(X1+X2+X3+X4)
        Y0=FOURTH*(Y1+Y2+Y3+Y4)
        Z0=FOURTH*(Z1+Z2+Z3+Z4)
        X1=X1-X0
        Y1=Y1-Y0
        Z1=Z1-Z0
        X2=X2-X0
        Y2=Y2-Y0
        Z2=Z2-Z0
        X3=X3-X0
        Y3=Y3-Y0
        Z3=Z3-Z0
        X4=X4-X0
        Y4=Y4-Y0
        Z4=Z4-Z0
        XS=X(1,I)-X0
        YS=X(2,I)-Y0
        ZS=X(3,I)-Z0
C
C        X12=X1*X1
C        X22=X2*X2
C        X32=X3*X3
C        X42=X4*X4 
C        Y12=Y1*Y1
C        Y22=Y2*Y2
C        Y32=Y3*Y3
C        Y42=Y4*Y4 
C        Z12=Z1*Z1 
C        Z22=Z2*Z2
C        Z32=Z3*Z3 
C        Z42=Z4*Z4 
C        XX=X12 + X22 + X32 + X42 
C        YY=Y12 + Y22 + Y32 + Y42 
C        ZZ=Z12 + Z22 + Z32 + Z42 
C        XY=X1*Y1 + X2*Y2 + X3*Y3 + X4*Y4 
C        YZ=Y1*Z1 + Y2*Z2 + Y3*Z3 + Y4*Z4 
C        ZX=Z1*X1 + Z2*X2 + Z3*X3 + Z4*X4
C        ZZZ=XX+YY
C        XXX=YY+ZZ
C        YYY=ZZ+XX 
C        XY2=XY*XY
C        YZ2=YZ*YZ
C        ZX2=ZX*ZX
C        DET= XXX*YYY*ZZZ - XXX*YZ2 - YYY*ZX2 - ZZZ*XY2 - 2.*XY*YZ*ZX
C        DET=1./DET
C        B1=ZZZ*YYY-YZ2
C        B2=XXX*ZZZ-ZX2
C        B3=YYY*XXX-XY2
C        C3=ZZZ*XY+YZ*ZX
C        C1=XXX*YZ+ZX*XY
C        C2=YYY*ZX+XY*YZ
C
C
        DET= DPARA(1,II)
        B1=DPARA(2,II)
        B2=DPARA(3,II)
        B3=DPARA(4,II)
        C1=DPARA(5,II)
        C2=DPARA(6,II)
        C3=DPARA(7,II)
C
        MGX = Y1*V(3,J1) + Y2*V(3,J2) + Y3*V(3,J3) + Y4*V(3,J4) 
     .      - Z1*V(2,J1) - Z2*V(2,J2) - Z3*V(2,J3) - Z4*V(2,J4)
        MGY = Z1*V(1,J1) + Z2*V(1,J2) + Z3*V(1,J3) + Z4*V(1,J4) 
     .      - X1*V(3,J1) - X2*V(3,J2) - X3*V(3,J3) - X4*V(3,J4)
        MGZ = X1*V(2,J1) + X2*V(2,J2) + X3*V(2,J3) + X4*V(2,J4) 
     .      - Y1*V(1,J1) - Y2*V(1,J2) - Y3*V(1,J3) - Y4*V(1,J4)
C
        MRX = Y1*A(3,J1) + Y2*A(3,J2) + Y3*A(3,J3) + Y4*A(3,J4) 
     .      - Z1*A(2,J1) - Z2*A(2,J2) - Z3*A(2,J3) - Z4*A(2,J4)
        MRY = Z1*A(1,J1) + Z2*A(1,J2) + Z3*A(1,J3) + Z4*A(1,J4) 
     .      - X1*A(3,J1) - X2*A(3,J2) - X3*A(3,J3) - X4*A(3,J4)
        MRZ = X1*A(2,J1) + X2*A(2,J2) + X3*A(2,J3) + X4*A(2,J4) 
     .      - Y1*A(1,J1) - Y2*A(1,J2) - Y3*A(1,J3) - Y4*A(1,J4)
C

        VRX=DET*(MGX*B1+MGY*C3+MGZ*C2)
        VRY=DET*(MGY*B2+MGZ*C1+MGX*C3)
        VRZ=DET*(MGZ*B3+MGX*C2+MGY*C1)
        ARX=DET*(MRX*B1+MRY*C3+MRZ*C2)
        ARY=DET*(MRY*B2+MRZ*C1+MRX*C3)
        ARZ=DET*(MRZ*B3+MRX*C2+MRY*C1)
C
C----------------------------------------------------
C       rotation velocity of the secondary node
C----------------------------------------------------
        IF (IRODDL == 1) THEN
          VR(1,I)=VRX
          VR(2,I)=VRY
          VR(3,I)=VRZ
          AR(1,I)=ARX
          AR(2,I)=ARY
          AR(3,I)=ARZ
        ENDIF
C----------------------------------------------------
C       velocity of the secondary node
C----------------------------------------------------
        V(1,I)=VMX + VRY*ZS - VRZ*YS
        V(2,I)=VMY + VRZ*XS - VRX*ZS
        V(3,I)=VMZ + VRX*YS - VRY*XS
        A(1,I)=AMX + ARY*ZS - ARZ*YS
        A(2,I)=AMY + ARZ*XS - ARX*ZS
        A(3,I)=AMZ + ARX*YS - ARY*XS
       ENDIF
      ENDDO
C
      RETURN
      END
