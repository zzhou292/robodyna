          thk(1:jlt) = max(thkn(1:jlt),em30)
!
          fact = onep414*dm
          visc(1:jlt) = fact*ssp(1:jlt)*sqrt(area(1:jlt))*dtinv*rho(1:jlt)
!
          for(1:jlt,1)=for(1:jlt,1)+visc(1:jlt)*(exx(1:jlt)+half*eyy(1:jlt))
          for(1:jlt,2)=for(1:jlt,2)+visc(1:jlt)*(eyy(1:jlt)+half*exx(1:jlt))
          for(1:jlt,3)=for(1:jlt,3)+visc(1:jlt)* exy(1:jlt) *third
!
          for(1:jlt,1)=for(1:jlt,1)*off(1:jlt)*dmg_glob_scale(1:jlt)
          for(1:jlt,2)=for(1:jlt,2)*off(1:jlt)*dmg_glob_scale(1:jlt)
          for(1:jlt,3)=for(1:jlt,3)*off(1:jlt)*dmg_glob_scale(1:jlt)
          for(1:jlt,4)=for(1:jlt,4)*off(1:jlt)*dmg_glob_scale(1:jlt)
          for(1:jlt,5)=for(1:jlt,5)*off(1:jlt)*dmg_glob_scale(1:jlt)
          mom(1:jlt,1)=mom(1:jlt,1)*off(1:jlt)*dmg_glob_scale(1:jlt)
          mom(1:jlt,2)=mom(1:jlt,2)*off(1:jlt)*dmg_glob_scale(1:jlt)
          mom(1:jlt,3)=mom(1:jlt,3)*off(1:jlt)*dmg_glob_scale(1:jlt)
