! SPDX-License-Identifier: AGPL-3.0-or-later
subroutine tr_reference(counts,interfaces,tied_mains,tied_rows,target,secondary,old_offsets,old_nodes, &
 old_inverse,old_mains,input_irtlm,input_history,out_offsets,out_nodes,out_inverse,out_mains, &
 out_irtlm,out_history,info) bind(C,name='type25_tied_removal_reference')
 use iso_c_binding
 use tr_buffer
 use tr_restart
 implicit none
 integer(c_int),intent(in)::counts(7),interfaces(4,*),tied_mains(4,*),tied_rows(2,*),target(5,*),secondary(*)
 integer(c_int),intent(in)::old_offsets(*),old_nodes(*),old_inverse(*),old_mains(*),input_irtlm(4,*)
 real(c_double),intent(in)::input_history(7,*)
 integer(c_int),intent(out)::out_offsets(*),out_nodes(*),out_inverse(*),out_mains(*),out_irtlm(4,*),info(3)
 real(c_double),intent(out)::out_history(7,*)
 type(intbuf_struct_),allocatable::buffers(:)
 integer,pointer::p(:,:)
 integer,allocatable::itab(:),nom_opt(:,:),nremov(:)
 integer::numnod,ninter,g,s,niface,oldextent,ni,i,j,k,m,r,mi,ri,used
 common /TR_COUNTS/numnod,ninter
 ! Counts/layout were checked by the bounded C++ adapter before crossing C.
 numnod=counts(1);g=counts(2);s=counts(3);niface=counts(4);oldextent=counts(7)
 ni=interfaces(1,niface)+1;ninter=ni
 allocate(ipari(200*ninter));ipari=0;p(1:200,1:ninter)=>ipari
 allocate(buffers(ninter),itab(numnod),nom_opt(4,ninter),nremov(ninter))
 nom_opt=0;nremov=0
 do i=1,numnod
  itab(i)=i
 enddo
 mi=0;ri=0
 do i=1,niface
  j=interfaces(1,i);m=interfaces(3,i);r=interfaces(4,i)
  p(4,j)=m;p(5,j)=r;p(7,j)=2;p(20,j)=28;nom_opt(1,j)=interfaces(2,i)
  allocate(buffers(j)%nsv(r),buffers(j)%irtlm(r),buffers(j)%irectm(4*m))
  do k=1,m
   buffers(j)%irectm(4*(k-1)+1:4*k)=tied_mains(:,mi+k)
  enddo
  do k=1,r
   buffers(j)%nsv(k)=tied_rows(1,ri+k);buffers(j)%irtlm(k)=tied_rows(2,ri+k)
  enddo
  mi=mi+m;ri=ri+r
 enddo
 p(4,ni)=g;p(5,ni)=s;p(7,ni)=25;p(58,ni)=0;p(62,ni)=oldextent;p(63,ni)=2;p(81,ni)=oldextent;p(83,ni)=1
 nom_opt(1,ni)=9001
 allocate(buffers(ni)%nsv(s),buffers(ni)%irtlm(4*s),buffers(ni)%irectm(4*g),buffers(ni)%mseglo(g))
 allocate(buffers(ni)%time_s(2*s),buffers(ni)%pene_old(5*s))
 allocate(buffers(ni)%kremnode(g+1),buffers(ni)%remnode(oldextent),buffers(ni)%kremnor(s+1),buffers(ni)%remnor(oldextent))
 buffers(ni)%s_kremnode=g+1;buffers(ni)%s_remnode=oldextent
 buffers(ni)%s_kremnor=s+1;buffers(ni)%s_remnor=oldextent
 buffers(ni)%remnode=0;buffers(ni)%remnor=0
 do i=1,g
  buffers(ni)%irectm(4*(i-1)+1:4*i)=target(1:4,i);buffers(ni)%mseglo(i)=target(5,i)
 enddo
 do i=1,s
  buffers(ni)%nsv(i)=secondary(i);buffers(ni)%irtlm(4*(i-1)+1:4*i)=input_irtlm(:,i)
  buffers(ni)%pene_old(5*(i-1)+1:5*i)=input_history(1:5,i)
  buffers(ni)%time_s(2*(i-1)+1:2*i)=input_history(6:7,i)
 enddo
 buffers(ni)%kremnode=old_offsets(1:g+1);buffers(ni)%kremnor=old_inverse(1:s+1)
 used=old_offsets(g+1)
 if(used>0)then
  buffers(ni)%remnode(1:used)=old_nodes(1:used);buffers(ni)%remnor(1:used)=old_mains(1:used)
 endif
 call tr_remn_i2op(ni,ni,p,buffers,itab,nom_opt,nremov,1,0)
 used=buffers(ni)%kremnode(g+1)
 info=[used,p(62,ni),p(82,ni)]
 out_offsets(1:g+1)=buffers(ni)%kremnode(1:g+1);out_inverse(1:s+1)=buffers(ni)%kremnor
 if(used>0)then
  out_nodes(1:used)=buffers(ni)%remnode(1:used);out_mains(1:used)=buffers(ni)%remnor(1:used)
 endif
 do i=1,s
  out_irtlm(:,i)=buffers(ni)%irtlm(4*(i-1)+1:4*i)
  out_history(1:5,i)=buffers(ni)%pene_old(5*(i-1)+1:5*i)
  out_history(6:7,i)=buffers(ni)%time_s(2*(i-1)+1:2*i)
 enddo
 deallocate(buffers,itab,nom_opt,nremov,ipari)
 nullify(p)
end subroutine
