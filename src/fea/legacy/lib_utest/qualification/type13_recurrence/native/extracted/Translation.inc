       DO I=1,NEL
         VX21  = VX2(I)-VX1(I)
         VY21  = VY2(I)-VY1(I)
         VZ21  = VZ2(I)-VZ1(I)
C
         EPXY = (VX21*EXY2(I)+VY21*EYY2(I)+VZ21*EZY2(I))*DT05
         EPXZ = (VX21*EXZ2(I)+VY21*EYZ2(I)+VZ21*EZZ2(I))*DT05
C
         X21  = (RX2(I)+RX1(I))
         Y21  = (RY2(I)+RY1(I))
         Z21  = (RZ2(I)+RZ1(I))
C
         RYAV1 = (X21*EXY2(I)+Y21*EYY2(I)+Z21*EZY2(I))
         RZAV1 = (X21*EXZ2(I)+Y21*EYZ2(I)+Z21*EZZ2(I))
C
         AT=EPXZ/MAX(AL2DP(I),EM30)
         AT=ATAN(AT)
         RYAV  = DT05 * (RYAV1) + TWO * AT
         AT=EPXY/MAX(AL2DP(I),EM30)
         AT=ATAN(AT)
         RZAV  = DT05 * (RZAV1) - TWO * AT
C
         DX(I) = ALDP(I) - X0DP(I)
         DY(I) = DYOLD(I) - RZAV * AL2DP(I)
         DZ(I) = DZOLD(I) + RYAV * AL2DP(I)
C
         CRIT(I) = ZERO
       ENDDO
