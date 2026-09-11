      DO I=1,NEL
        EPLA(I)=ZERO
        XM(I)=GEO(1,MGN(I))
        XK(I)=GEO(3,MGN(I))
        XC(I)=GEO(4,MGN(I))
        YK(I)=GEO(10,MGN(I))
        YC(I)=GEO(11,MGN(I))
        XKA  =GEO(41,MGN(I))*XK(I)
        YKA  =GEO(45,MGN(I))*YK(I)
        XKM(I)= MAX(XKA,YKA)
        HX(I) = GEO(141,MGN(I))
        HY(I) = GEO(142,MGN(I))

        XH(I)= MAX(HX(I),HY(I))

        XCM(I)= MAX(XC(I),YC(I))
        XCM(I)= XCM(I)+XH(I)

        XKR(I)= YKA*ALDP(I)*ALDP(I)
        XCR(I)= (YC(I)+ HY(I))*ALDP(I)* ALDP(I)
      ENDDO
