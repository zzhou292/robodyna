! SPDX-License-Identifier: AGPL-3.0-or-later
! Serial qualification-only composition. Original routines own every numerical
! margin, connected-neighbor traversal and transposition operation.
subroutine rd_search_startup(counts,x,irect,roles,nsv,gap_s,stfn,gap_m, &
    scalars,extent,krem,rem,kremnor,remnor,icont) bind(C)
 use iso_c_binding
 use intbufdef_mod
 use get_list_remnode_mod
 implicit none
 integer(c_int),intent(in)::counts(5),irect(4,counts(2)),roles(counts(2)),nsv(counts(3))
 real(c_double),intent(in)::x(3,counts(1)),gap_s(counts(3)),stfn(counts(3)),gap_m(counts(2))
 real(c_double),intent(out)::scalars(5),extent(counts(4))
 integer(c_int),intent(out)::krem(counts(2)+1),rem(counts(2)*counts(3))
 integer(c_int),intent(out)::kremnor(counts(3)+1),remnor(counts(2)*counts(3)),icont(counts(3))
 integer::n,g,s,p,i,self,nrem,imem,ipari(105),itab(counts(1)),nbin(counts(3)),mbin(counts(2))
 integer::knod(counts(1)+1),nod(4*counts(2)),tag(counts(1))
 real(c_double)::gap,gapsmax,gapmmax,gapmin,gapmax,minseg,gapsnode(counts(1)),gaplnode(counts(1))
 real(c_double)::gapsl(counts(3)),gapml(counts(2)),zero,large
 type(intbuf_struct_)::buffer
 ! Explicit interface for the separate BIND(C) scalar reference entry.
 interface
  subroutine rd_startup_multiplier(nodes,value) bind(C)
   use iso_c_binding
   integer(c_int),intent(in)::nodes
   real(c_double),intent(out)::value
  end subroutine
 end interface
 n=counts(1);g=counts(2);s=counts(3);p=counts(4);zero=0;large=1e20_c_double*1e10_c_double
 gapmin=zero;gapmax=large;gapsmax=zero;gapmmax=zero
 do i=1,s
  gapsmax=max(gapsmax,gap_s(i))
 end do
 do i=1,p
  gapmmax=max(gapmmax,gap_m(i))
 end do
 gap=gapsmax+gapmmax
 call rd_startup_multiplier(n,scalars(1))
 call rd_startup_margin(n,g,s,x,irect,roles,nsv,stfn,gap,scalars(1),scalars(2),scalars(3))
 call rd_startup_extent(n,p,x,irect,counts(5),extent,scalars(4))
 ipari=0;ipari(7)=25;ipari(21)=1;ipari(63)=2
 do i=1,n
  itab(i)=i
 end do
 nbin=0;mbin=0;knod=0;nod=0;tag=0;gapsnode=0;gaplnode=0;gapsl=0;gapml=0
 krem=0;rem=0;kremnor=0;remnor=0;nrem=0;imem=0;minseg=large
 call rd_initial_flags(s,icont)
 buffer%s_remnode=g*s;allocate(buffer%remnode(g*s));buffer%remnode=0
 call I7REMNODE_INIT(self,25,x,g,irect,nsv,s,n,itab,gap_s,gap_m,gapmin,gapmax, &
     gapsl,gapml,1,krem,rem,gap,zero,nrem,1,nbin,mbin,ipari,imem,gapmmax,gapsmax, &
     zero,zero,knod,nod,tag,gapsnode,gaplnode,minseg)
 if(self/=0)then
  call get_list_remnode(g,1,n,105,irect,krem,knod,nod,tag,ipari,gapmin,gapmax,gap,zero, &
      gapsmax,zero,minseg,zero,x,gap_m,gapml,gapsnode,gaplnode,buffer)
  rem=buffer%remnode
  call I25REMNOR(g,irect,nsv,s,n,krem,rem,kremnor,remnor,ipari,tag)
 endif
 scalars(5)=minseg
end subroutine
