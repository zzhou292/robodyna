        DT05 = HALF*DT1
        IF(IDYNA>0) DT05=(DY_G-ONE)*DT1
        VG(1)=VR(1,M)+AR(1,M)*DT05
        VG(2)=VR(2,M)+AR(2,M)*DT05
        VG(3)=VR(3,M)+AR(3,M)*DT05
        VI(1)=RBY(1)*VG(1)+RBY(2)*VG(2)+RBY(3)*VG(3)
        VI(2)=RBY(4)*VG(1)+RBY(5)*VG(2)+RBY(6)*VG(3)
        VI(3)=RBY(7)*VG(1)+RBY(8)*VG(2)+RBY(9)*VG(3)
C
        ENROTT= - ( VG(1)*VG(1)
     .           + VG(2)*VG(2)
     .           + VG(3)*VG(3))*IN(M)*WEIGHT_MD(M)
     .         + ( RBY(10)*VI(1)*VI(1)
     .           + RBY(11)*VI(2)*VI(2)
     .           + RBY(12)*VI(3)*VI(3))*WEIGHT_MD(M)
        ENCINT=ZERO
        XMASST=ZERO
