! SPDX-License-Identifier: AGPL-3.0-or-later
module startup_native_names
  implicit none
  integer,parameter::nchartitle=128
end module
module startup_native_messages
  implicit none
  integer,parameter::aninfo_blind_2=0,msgwarning=0,msgerror=1
contains
  subroutine ancmsg(msgid,msgtype,anmode,i1,i2,i3)
    integer,intent(in)::msgid,msgtype,anmode
    integer,optional,intent(in)::i1,i2,i3
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
