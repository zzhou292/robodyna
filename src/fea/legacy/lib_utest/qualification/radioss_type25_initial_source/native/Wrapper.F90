subroutine initial_inventory_native(counts,x,irect,msegtyp,nsv,msr,icode,iskew,stf,stfn,gap_m,gap_s,gap_n, &
    ielem_m,ixs,iparts,knod2els,nod2els,kremnode,remnode,controls,found,cand_n,cand_e,diagnostics) &
    bind(C,name='initial_inventory_native')
 use iso_c_binding
 use intbufdef_mod
 use names_and_titles_mod
 use tri7box
 implicit none
 integer(c_int),intent(in)::counts(6),irect(4,*),msegtyp(*),nsv(*),msr(*),icode(*),iskew(*)
 integer(c_int),intent(in)::ielem_m(2,*),ixs(11,*),iparts(*),knod2els(*),nod2els(*),kremnode(*),remnode(*)
 real(c_double),intent(in)::x(3,*),stf(*),stfn(*),gap_m(*),gap_s(*),controls(3)
 real(c_double),intent(inout)::gap_n(4,*)
 integer(c_int),intent(out)::found,cand_n(*),cand_e(*)
 real(c_double),intent(out)::diagnostics(5)
 integer::numnod,numels,numels8,numels10,numels16,numels20,npart,numnor,ninter25,nsnt25,nrtmx25
 common /INITIAL_INVENTORY_COUNTS/ numnod,numels,numels8,numels10,numels16,numels20,npart,numnor,ninter25,nsnt25,nrtmx25
 type(intbuf_struct_)::buffer
 integer::n,g,s,nmn,ns,i,ipari(200),nbi(counts(3)),mbi(counts(2)),empty(1)
 real(c_double)::gapmin,gapmax,marge,dist,tzinf,maxbox,minbox,gls(counts(3)),glm(counts(2))
 character(len=NCHARTITLE)::title
 n=counts(1);g=counts(2);s=counts(3);nmn=counts(4);ns=counts(5)
 if(n<1.or.n>512.or.g<1.or.g>320.or.s<1.or.s>256.or.ns<0.or.ns>64)error stop 'Native inventory bounds'
 numnod=n;numels=ns;numels8=ns;numels10=0;numels16=0;numels20=0;npart=0
 do i=1,ns
  npart=max(npart,iparts(i))
 enddo
 allocate(buffer%cand_n(g*s),buffer%cand_e(g*s))
 buffer%cand_n=0;buffer%cand_e=0;ipari=0;ipari(18)=s;ipari(23)=g
 nbi=0;mbi=0;empty=0;gls=0;glm=0;found=0
 title='Independent bounded initial inventory';inivoxel=1
 call I25BUC_VOX1(x,irect,nsv,controls(1),nmn,g,s,buffer,controls(2),found,dist,tzinf,maxbox,minbox,msr, &
   stf,stfn,1,gap_s,gap_m,1,gapmin,gapmax,5,gls,glm,marge,1,title,nbi,mbi,1, &
   msegtyp,gap_n,controls(3),iparts,knod2els,nod2els,kremnode,remnode, &
   ixs,empty,empty,empty,icode,iskew,0._c_double,0._c_double,g,counts(6)>0,ielem_m,1,200,ipari)
 if(found<0.or.found>g*s)error stop 'Invalid native inventory count'
 cand_n(1:found)=buffer%cand_n(1:found);cand_e(1:found)=buffer%cand_e(1:found)
 diagnostics=[marge,dist,tzinf,maxbox,real(inivoxel-1,c_double)]
end subroutine

subroutine initial_prepared_native(counts,irtlm) bind(C,name='initial_prepared_native')
 use iso_c_binding
 use intbufdef_mod
 use front_mod
 implicit none
 integer(c_int),intent(in)::counts(2)
 integer(c_int),intent(inout)::irtlm(4,*)
 type(intbuf_struct_)::buffer(1)
 type(intersurfp)::intercep(3,1)
 integer::ipari(200,1),nrtmt,i
 if(counts(1)<1.or.counts(1)>320.or.counts(2)<1.or.counts(2)>256)error stop 'Native prepare bounds'
 allocate(buffer(1)%irtlm(4*counts(2)),intercep(1,1)%p(counts(1)))
 do i=1,counts(2)
  buffer(1)%irtlm(4*i-3:4*i)=irtlm(1:4,i)
 enddo
 intercep(1,1)%p=1 ! Actual selected serial SET_INTERCEP partition, not a guessed local map.
 ipari=0;ipari(7,1)=25;ipari(4,1)=counts(1);ipari(5,1)=counts(2)
 call PREPARE_INT25(buffer,ipari,intercep,nrtmt)
 do i=1,counts(2)
  irtlm(1:4,i)=buffer(1)%irtlm(4*i-3:4*i)
 enddo
 deallocate(intercep(1,1)%p)
end subroutine
