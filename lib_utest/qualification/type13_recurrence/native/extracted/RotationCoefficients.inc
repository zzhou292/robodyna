      DO I=1,NEL
        XIN(I)=GEO(9,MGN(I))
        XK(I)=GEO(19,MGN(I))
        XC(I)=GEO(20,MGN(I))
        YK(I)=GEO(23,MGN(I))
        YC(I)=GEO(24,MGN(I))
        ZK(I)=GEO(27,MGN(I))
        ZC(I)=GEO(28,MGN(I))
        HX(I) =  GEO(144,MGN(I))
        HY(I) =  GEO(145,MGN(I))
        HZ(I) =  GEO(146,MGN(I))

        XHR(I)= MAX(HX(I),HY(I),HZ(I))

        XKR(I)= MAX(XK(I)*GEO(53,MGN(I)),
     .              YK(I)*GEO(57,MGN(I)),
     .              ZK(I)*GEO(61,MGN(I)))+XKR(I)
        XCR(I)= MAX(XC(I),YC(I),ZC(I)) + XHR(I) +XCR(I)+XH(I)
      ENDDO
