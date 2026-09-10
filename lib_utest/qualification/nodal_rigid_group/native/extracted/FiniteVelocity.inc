          call cross_product(vr,lsm,vs)
          vrm(1:3) = vr(1:3)*dt
          ang2 = vrm(1)*vrm(1)+vrm(2)*vrm(2)+vrm(3)*vrm(3)
          vs2 = (vs(1)*vs(1)+vs(2)*vs(2)+vs(3)*vs(3))*dt*dt  !ds
          if ( ang2 > em06 .and. vs2 > em08) then
            angelv = sqrt(ang2)
            norm = one/max(em20,angelv)
            localz(1:3) = norm*vrm(1:3)
            norm = dt/sqrt(vs2)
            localx(1:3) = norm*vs(1:3)
            call cross_product(localz,localx,localy)
            trans(1,1:3) = localx(1:3)
            trans(2,1:3) = localy(1:3)
            trans(3,1:3) = localz(1:3)
            lsmlocal(1:3) = trans(1:3,1)*lsm(1)+trans(1:3,2)*lsm(2)+trans(1:3,3)*lsm(3)
            rz(1,1) = cos(angelv)
            rz(1,2) = sin(angelv)
            rz(2,1) = -rz(1,2)
            rz(2,2) = rz(1,1)
            rz(3,3) = one
            rz(1:2,3) = zero
            rz(3,1:2) = zero
            lsmltr(1:3) = rz(1,1:3)*lsmlocal(1)+rz(2,1:3)*lsmlocal(2)+rz(3,1:3)*lsmlocal(3)
            lsmgtr(1:3) = trans(1,1:3)*lsmltr(1)+trans(2,1:3)*lsmltr(2)+trans(3,1:3)*lsmltr(3)
            vs(1:3) = (lsmgtr(1:3) - lsm(1:3))/dt   ! velocity now
          end if
