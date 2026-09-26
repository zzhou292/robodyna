! Private serial environment; every numerical donor is compiled separately.
module precision_mod
 use iso_c_binding
 implicit none
 integer,parameter::WP=c_double
end module
module names_and_titles_mod
 implicit none
 integer,parameter::NCHARTITLE=128
end module
module message_mod
 implicit none
 integer,parameter::MSGWARNING=0,ANINFO_BLIND_2=0,ANINFO=0,MSGERROR=0
contains
 subroutine ANCMSG(MSGID,MSGTYPE,ANMODE,I1,C1)
  integer,intent(in)::MSGID,MSGTYPE,ANMODE
  integer,intent(in),optional::I1
  character(*),intent(in)::C1
  error stop 'Unexpected initial inventory diagnostic'
 end subroutine
end module
module element_mod
 implicit none
 integer,parameter::NIXS=11
end module
module intbufdef_mod
 implicit none
 type::intbuf_struct_
  integer,allocatable::cand_n(:),cand_e(:),irtlm(:)
 end type
end module
module front_mod
 implicit none
 type::intersurfp
  integer,pointer::p(:)=>null()
 end type
end module
module my_dealloc_mod
 use iso_c_binding
 implicit none
 interface my_dealloc
  module procedure int_vector,int_matrix,real_vector
 end interface
contains
 subroutine int_vector(a)
  integer,allocatable,intent(inout)::a(:)
  if(allocated(a))deallocate(a)
 end subroutine
 subroutine int_matrix(a)
  integer,allocatable,intent(inout)::a(:,:)
  if(allocated(a))deallocate(a)
 end subroutine
 subroutine real_vector(a)
  real(c_double),allocatable,intent(inout)::a(:)
  if(allocated(a))deallocate(a)
 end subroutine
end module
module my_alloc_mod
 use iso_c_binding
 use my_dealloc_mod,only:my_dealloc
 implicit none
 private
 public::my_alloc,my_dealloc
 interface my_alloc
  module procedure int_vector,int_matrix,real_vector
 end interface
contains
 subroutine int_vector(a,n,name,stat)
  integer,allocatable,intent(inout)::a(:)
  integer,intent(in)::n
  character(*),intent(in)::name
  integer,intent(out),optional::stat
  integer::error
  allocate(a(n),stat=error)
  if(present(stat))stat=error
  if(.not.present(stat).and.error/=0)error stop 'Native oracle allocation failed'
 end subroutine
 subroutine int_matrix(a,n,m,name)
  integer,allocatable,intent(inout)::a(:,:)
  integer,intent(in)::n,m
  character(*),intent(in)::name
  allocate(a(n,m))
 end subroutine
 subroutine real_vector(a,n,name)
  real(c_double),allocatable,intent(inout)::a(:)
  integer,intent(in)::n
  character(*),intent(in)::name
  allocate(a(n))
 end subroutine
end module
module my_move_alloc_mod
 implicit none
contains
 subroutine my_move_alloc(a,b,name)
  integer,allocatable,intent(inout)::a(:),b(:)
  character(*),intent(in)::name
  call move_alloc(a,b)
 end subroutine
end module
module array_mod
 implicit none
 type::array_type_int_1d
  integer::size_int_array_1d=0
  integer,allocatable::int_array_1d(:)
 end type
contains
 subroutine alloc_1d_array(a)
  type(array_type_int_1d),intent(inout)::a
  allocate(a%int_array_1d(a%size_int_array_1d))
 end subroutine
 subroutine dealloc_1d_array(a)
  type(array_type_int_1d),intent(inout)::a
  if(allocated(a%int_array_1d))deallocate(a%int_array_1d)
 end subroutine
end module
integer function omp_get_thread_num()
 omp_get_thread_num=0
end function
integer function omp_get_num_threads()
 omp_get_num_threads=1
end function
integer function bitget(word,bit)
 integer,intent(in)::word,bit
 bitget=ibits(word,bit,1)
end function
subroutine upgrade_multimp(nin,multimp,buffer)
 use intbufdef_mod
 implicit none
 integer,intent(in)::nin,multimp
 type(intbuf_struct_),intent(inout)::buffer
 error stop 'Native inventory exceeded complete G*S capacity'
end subroutine
