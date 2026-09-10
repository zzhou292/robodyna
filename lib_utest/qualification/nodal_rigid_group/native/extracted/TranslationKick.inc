      DO N=1,NUMNOD! NODFT,NODLT
        V(1,N)=V(1,N)+DT12*A(1,N)
        V(2,N)=V(2,N)+DT12*A(2,N)
        V(3,N)=V(3,N)+DT12*A(3,N)
        A(1,N)=ZERO
        A(2,N)=ZERO
        A(3,N)=ZERO
      ENDDO
