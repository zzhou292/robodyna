      DO I=1,NSN
        N = NOD(I)
C
C       Reaction force is also valid with zero mass or inertia
        R(1,I)  =  MS(N)*A(1,N)  - FREAC(1,N)
        R(2,I)  =  MS(N)*A(2,N)  - FREAC(2,N)
        R(3,I)  =  MS(N)*A(3,N)  - FREAC(3,N)
C
        RR(1,I) =  IN(N)*AR(1,N) - FREAC(4,N)
        RR(2,I) =  IN(N)*AR(2,N) - FREAC(5,N)
        RR(3,I) =  IN(N)*AR(3,N) - FREAC(6,N)
C
      ENDDO
