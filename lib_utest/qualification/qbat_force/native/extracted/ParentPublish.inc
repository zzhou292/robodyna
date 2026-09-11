          nindx = 0
          if (ixfem == 0) then
            do i=1,jlt
              if (off(i) == four_over_5 .and. ioff_duct(i) == 0 .or.&   ! rupture /fail
              &off(i) > zero   .and. off_old(i) < em01 .or.&   ! rupture progressive law 2,22,25
              &off(i) > zero   .and. off(i) < one .and. dmg_flag == 1 .and.&
              &dmg_glob_scale(i) < em02) then
                off(i) = zero
                ! printing message not always mandatory
                if (print_fail(i)) then
                  nindx  = nindx + 1
                  indx(nindx) = i
                endif
              endif
            enddo
          endif
