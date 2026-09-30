          do i=1,jlt
            vol2 = half*vol0(i)
            if (mtn == 22) vol2 = vol2*off(i)
            eint(i,1) = eint(i,1) + degmb(i)*vol2
            eint(i,2) = eint(i,2) + degfx(i)*thk0(i)*vol2
          enddo
