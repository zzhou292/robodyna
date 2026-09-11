          degmb(1:jlt) = degmb(1:jlt)+ for(1:jlt,1)*exx(1:jlt)+for(1:jlt,2)*eyy(1:jlt)&
          &+ for(1:jlt,3)*exy(1:jlt)+for(1:jlt,4)*eyz(1:jlt)+ for(1:jlt,5)*exz(1:jlt)
          degfx(1:jlt) = degfx(1:jlt)+ mom(1:jlt,1)*kxx(1:jlt)+mom(1:jlt,2)*kyy(1:jlt)+mom(1:jlt,3)*kxy(1:jlt)
