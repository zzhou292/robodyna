                  do i = 1,jlt
                    dpla(i) = lbuf%pla(i) - pla0(i)
                    vm0(i) = sqrt(sigoxx(i)*sigoxx(i) + sigoyy(i)*sigoyy(i) -   &
                      sigoxx(i)*sigoyy(i) + three*sigoxy(i)*sigoxy(i))
                    vm(i)  = sqrt(signxx(i)*signxx(i) + signyy(i)*signyy(i) -   &
                      signxx(i)*signyy(i) + three*signxy(i)*signxy(i))
                    gbuf%wpla(i) = gbuf%wpla(i) +                               &
                      half*(vm0(i) + vm(i))*dpla(i)*thklyl(i)*area(i)
                  enddo
