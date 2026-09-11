            do i=1,nel
              dvv  = max(one,abs(dvx(i)/d(i)))
              dfac = ak(i) + b(i) * log(dvv) + ee(i)*gx(i)
              fx(i)= ( dfac*fx(i) + xc(i)*dvx(i) + gf3(i)*gx2(i) ) *off(i)
              e(i) = e(i) + (dx(i)-dxold(i)) * (fx(i)+fold(i)) * half
            end do
