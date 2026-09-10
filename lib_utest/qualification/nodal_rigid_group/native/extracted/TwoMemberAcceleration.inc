          DO I=1,NSN
            N = NOD(I)
            AR(1:3,N)= (VG(1:3)-VR(1:3,N)) * USDT
            A(1:3,N)=  A(1:3,M)+(V(1:3,M)-V(1:3,N))*USDT
C
            LSM(1:3) = X(1:3,N)-X(1:3,M)
            CALL VELROT_EXPLICIT(VG, LSM,VS,DT12)
C  
            VX1=VS(1)
            VX2=VS(2)
            VX3=VS(3)

            A(1,N)= A(1,N) +
     .         (VX1+HALF*DT2*(VG(2)*VX3-VG(3)*VX2))*USDT
            A(2,N)= A(2,N) +
     .         (VX2+HALF*DT2*(VG(3)*VX1-VG(1)*VX3))*USDT
            A(3,N)= A(3,N) +
     .         (VX3+HALF*DT2*(VG(1)*VX2-VG(2)*VX1))*USDT
          ENDDO
