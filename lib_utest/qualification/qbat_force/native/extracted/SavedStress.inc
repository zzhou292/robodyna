              do i=1,jlt
                ! ij(k) = nel*(k-1)
                lbuf%sig(nel*(1-1)+i) = signxx(i) * sigoff(i)
                lbuf%sig(nel*(2-1)+i) = signyy(i) * sigoff(i)
                lbuf%sig(nel*(3-1)+i) = signxy(i) * sigoff(i)
                lbuf%sig(nel*(4-1)+i) = signyz(i) * sigoff(i)
                lbuf%sig(nel*(5-1)+i) = signzx(i) * sigoff(i)
              enddo
