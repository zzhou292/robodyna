            N = NOD(I)
C
            MAS=MS(N)*WEIGHT_MD(N)
            WEWE2 = (1-WEIGHT_MD(N))*WEIGHT(N)
            VG(1)=V(1,N)+A(1,N)*DT05
            VG(2)=V(2,N)+A(2,N)*DT05
            VG(3)=V(3,N)+A(3,N)*DT05
            ENCINT=ENCINT - ( VG(1)*VG(1)
     .                   + VG(2)*VG(2)
     .                   + VG(3)*VG(3))*MAS
            ENCIN2T=ENCIN2T - ( VG(1)*VG(1)
     .                   + VG(2)*VG(2)
     .                   + VG(3)*VG(3))*MS(N)*WEWE2      
            XMOMTT=XMOMTT-VG(1)*MAS
            YMOMTT=YMOMTT-VG(2)*MAS
            ZMOMTT=ZMOMTT-VG(3)*MAS
            VG(1)=VR(1,N)+AR(1,N)*DT05
            VG(2)=VR(2,N)+AR(2,N)*DT05
            VG(3)=VR(3,N)+AR(3,N)*DT05
            ENROTT=ENROTT - ( VG(1)*VG(1)
     .                   + VG(2)*VG(2)
     .                   + VG(3)*VG(3))*IN(N)*WEIGHT_MD(N)
            ENROT2T=ENROT2T - ( VG(1)*VG(1)
     .                   + VG(2)*VG(2)
     .                   + VG(3)*VG(3))*IN(N)*WEWE2      
            XMASST=XMASST-MAS
