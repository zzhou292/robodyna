       RX = DT*VR(1)
       RY = DT*VR(2)
       RZ = DT*VR(3)
C
       RX2 = RX*RX
       RY2 = RY*RY
       RZ2 = RZ*RZ
C
       R2 = MAX(EM10,RX2+RY2+RZ2)
       R  = SQRT(R2)
C
       C = COS(R)
       CM1 = (1-C)
       C = C*R2
       S = SIN(R)*R
       SZ = RZ*S
       CZ = RZ*CM1
C
       E11 = RX2*CM1 + C
       E22 = RY2*CM1 + C
       E12 = RX*RY*CM1
       E21 = E12 - SZ
       E12 = E12 + SZ
       E13 = RX*CZ - RY*S
       E23 = RY*CZ + RX*S
C
       BL11 = RBY(1)*E11 + RBY(4)*E12 + RBY(7)*E13
       BL21 = RBY(2)*E11 + RBY(5)*E12 + RBY(8)*E13
       BL31 = RBY(3)*E11 + RBY(6)*E12 + RBY(9)*E13
C
       BL12 = RBY(1)*E21 + RBY(4)*E22 + RBY(7)*E23
       BL22 = RBY(2)*E21 + RBY(5)*E22 + RBY(8)*E23
       BL32 = RBY(3)*E21 + RBY(6)*E22 + RBY(9)*E23
C
       BN = SQRT(BL11*BL11+BL21*BL21+BL31*BL31)
       BL11=BL11/BN
       BL21=BL21/BN
       BL31=BL31/BN
       BN = SQRT(BL12*BL12+BL22*BL22+BL32*BL32)
       BL12=BL12/BN
       BL22=BL22/BN
       BL32=BL32/BN
C
       RBY(1) = BL11
       RBY(2) = BL21
       RBY(3) = BL31
C
       RBY(4) = BL12
       RBY(5) = BL22
       RBY(6) = BL32
C
       RBY(7) = BL21*BL32 - BL31*BL22
       RBY(8) = BL31*BL12 - BL11*BL32
       RBY(9) = BL11*BL22 - BL21*BL12
