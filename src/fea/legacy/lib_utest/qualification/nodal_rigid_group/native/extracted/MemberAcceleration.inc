          DO I=1,NSN
            N = NOD(I)
            AR(1,N)= (VG(1)-VR(1,N)) * USDT
            AR(2,N)= (VG(2)-VR(2,N)) * USDT
            AR(3,N)= (VG(3)-VR(3,N)) * USDT
C
            V1X2=VG(1)*(X(2,N)-X(2,M))
            V2X1=VG(2)*(X(1,N)-X(1,M))
            V2X3=VG(2)*(X(3,N)-X(3,M))
            V3X2=VG(3)*(X(2,N)-X(2,M))
            V3X1=VG(3)*(X(1,N)-X(1,M))
            V1X3=VG(1)*(X(3,N)-X(3,M))
C  
            VX1=V2X3-V3X2
            VX2=V3X1-V1X3
            VX3=V1X2-V2X1

            A(1,N)= AM1 +
     .        (V(1,M)+VX1+HALF*DT2*(VG(2)*VX3-VG(3)*VX2)-V(1,N))*USDT
            A(2,N)= AM2 +
     .        (V(2,M)+VX2+HALF*DT2*(VG(3)*VX1-VG(1)*VX3)-V(2,N))*USDT
            A(3,N)= AM3 +
     .        (V(3,M)+VX3+HALF*DT2*(VG(1)*VX2-VG(2)*VX1)-V(3,N))*USDT
          ENDDO
