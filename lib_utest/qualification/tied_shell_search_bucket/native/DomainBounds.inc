C--------------------------------
      XMIN=EP30
      XMAX=-EP30
      YMIN=EP30
      YMAX=-EP30
      ZMIN=EP30
      ZMAX=-EP30
C
      DO I=1,NMN
        J=MSR(I) 
        XMIN= MIN(XMIN,X(1,J))
        YMIN= MIN(YMIN,X(2,J))
        ZMIN= MIN(ZMIN,X(3,J))
        XMAX= MAX(XMAX,X(1,J))
        YMAX= MAX(YMAX,X(2,J))
        ZMAX= MAX(ZMAX,X(3,J))
      ENDDO
      ! save the main domain bounds (without the secondary part of the interface)
      margin = zero
      do i=1,nrtm
        margin = max(margin,segment_data(i,2))
      enddo
      margin = max(margin,tzinf05)
      bound(1)=xmin - margin
      bound(2)=ymin - margin
      bound(3)=zmin - margin
      bound(4)=xmax + margin
      bound(5)=ymax + margin
      bound(6)=zmax + margin
 

      distance(1:3) = bound(4:6) - bound(1:3)
