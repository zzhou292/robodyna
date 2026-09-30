! SPDX-License-Identifier: AGPL-3.0-or-later
module tr_buffer
 use iso_c_binding
 implicit none
 type::intbuf_struct_
  integer::s_kremnode=0,s_remnode=0,s_kremnor=0,s_remnor=0,s_kremnode_edg=0,s_kremnode_e2s=0
  integer,allocatable::nsv(:),irectm(:),irtlm(:),mseglo(:),kremnode(:),remnode(:),kremnor(:),remnor(:)
  integer,allocatable::kremnode_edg(:),kremnode_e2s(:)
  real(c_double),allocatable::time_s(:),pene_old(:)
 end type
end module
module tr_restart
 implicit none
 integer,allocatable,target::ipari(:)
end module
module tr_intbuf
end module
module tr_names
 integer,parameter::nchartitle=128
end module
module tr_format
 character(*),parameter::fmw_10i='(10I10)'
end module
module tr_messages
 implicit none
 integer,parameter::msgwarning=1,aninfo_blind_1=1
contains
 subroutine ancmsg(msgid,msgtype,anmode,i1,c1,i2,i3)
  integer,intent(in)::msgid,msgtype,anmode,i1,i2,i3
  character(*),intent(in)::c1
  if(msgid/=1053.or.msgtype/=msgwarning.or.anmode/=aninfo_blind_1)error stop 'Unexpected native diagnostic'
 end subroutine
end module
module tr_memory
 use iso_c_binding
 implicit none
 private
 public::my_alloc,my_dealloc
 interface my_alloc
  module procedure allocate_i1,allocate_i2
 end interface
 interface my_dealloc
  module procedure deallocate_i1,deallocate_i2
 end interface
contains
 subroutine allocate_i1(a,n,name)
  integer,allocatable,intent(inout)::a(:)
  integer,intent(in)::n
  character(*),intent(in)::name
  allocate(a(n))
 end subroutine
 subroutine allocate_i2(a,n,m,name)
  integer,allocatable,intent(inout)::a(:,:)
  integer,intent(in)::n,m
  character(*),intent(in)::name
  allocate(a(n,m))
 end subroutine
 subroutine deallocate_i1(a)
  integer,allocatable,intent(inout)::a(:)
  if(allocated(a))deallocate(a)
 end subroutine
 subroutine deallocate_i2(a)
  integer,allocatable,intent(inout)::a(:,:)
  if(allocated(a))deallocate(a)
 end subroutine
end module
module tr_move
 implicit none
contains
 subroutine my_move_alloc(a,b,name)
  integer,allocatable,intent(inout)::a(:),b(:)
  character(*),intent(in)::name
  call move_alloc(a,b)
 end subroutine
end module
subroutine tr_fretitl2(title,source,extent)
 implicit none
 character(*),intent(out)::title
 integer,intent(in)::source(*),extent
 title='Bounded serial tied removal qualification'
end subroutine
subroutine tr_edge_unsupported()
 error stop 'Edge TYPE25 path is not admitted by this wrapper'
end subroutine
