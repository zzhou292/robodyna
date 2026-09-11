          vol0(1:jlt)   = area(1:jlt)*thk0(1:jlt)
          thkn(1:jlt)   = thk(1:jlt)
          for(1:jlt,1)  = zero
          for(1:jlt,2)  = zero
          for(1:jlt,3)  = zero
          for(1:jlt,4)  = zero
          for(1:jlt,5)  = zero
          mom(1:jlt,1)  = zero
          mom(1:jlt,2)  = zero
          mom(1:jlt,3)  = zero
          yld(1:jlt)    = zero
          seq0(1:jlt)   = zero
          if (flag_law2 .or. flag_law25 .or. (.not.flag_zcfac)) sigy(1:jlt)   = zero
          if(flag_zcfac .and. mtn /= 22) zcfac(1:jlt,1)= zero
          zcfac(1:jlt,2)= zero
          if(flag_zcfac) zcfac(1:jlt,2)= one
          etse(1:jlt)   = one
