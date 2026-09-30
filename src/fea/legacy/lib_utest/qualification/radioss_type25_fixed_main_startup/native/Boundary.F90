! SPDX-License-Identifier: AGPL-3.0-or-later
module startup_native_observation
  use iso_c_binding
  implicit none
  integer::warning_count=0,warning_nodes(2)=0,selector_calls=0,score_count=0
  real(c_double)::angles(320)=0,sides(320)=0
contains
  subroutine clear_observations()
    warning_count=0;warning_nodes=0;selector_calls=0;score_count=0;angles=0;sides=0
  end subroutine
end module
module startup_native_names
  implicit none
  integer,parameter::nchartitle=128
end module
module startup_native_messages
  use startup_native_observation
  implicit none
  integer,parameter::aninfo_blind_2=0,msgwarning=0,msgerror=1
contains
  subroutine ancmsg(msgid,msgtype,anmode,i1,i2,i3)
    integer,intent(in)::msgid,msgtype,anmode
    integer,optional,intent(in)::i1,i2,i3
    if(msgid==1245.and.msgtype==msgwarning)then
      if(.not.present(i2).or..not.present(i3))error stop 'Missing native warning source edge'
      if(warning_count==0)warning_nodes=[i2,i3]
      warning_count=warning_count+1
      return
    endif
    error stop "Unsupported native topology diagnostic reached by qualification"
  end subroutine
end module
module startup_native_mpi
  use iso_c_binding
  implicit none
  ! These fields permit the complete source's foreign signature to compile.
  ! NSPMD is fixed1 by the local wrapper, so no foreign buffers are consumed.
  type::mpi_comm_nor_struct
    integer::recv_rq(1),send_rq(1),irindex(1),isindex(1),iad_recv(1)
    integer::nbirecv=0,nbisend=0
    real(c_float)::recv_buf(1),send_buf(1)
  end type
end module
subroutine startup_native_barrier()
end subroutine
subroutine startup_native_foreign_exchange()
  error stop "Foreign normal exchange is outside local startup qualification"
end subroutine

! Read-only observation at the original per-candidate scalar assignment.
subroutine startup_native_score(index,angle,side)
  use iso_c_binding
  use startup_native_observation
  implicit none
  integer,intent(in)::index
  real(c_double),intent(in)::angle,side
  if(index<1.or.index>320)error stop 'Native selector observation exceeds fixture cap'
  if(index==1)then
    selector_calls=selector_calls+1
    score_count=0;angles=0;sides=0
  endif
  score_count=index;angles(index)=angle;sides(index)=side
end subroutine
