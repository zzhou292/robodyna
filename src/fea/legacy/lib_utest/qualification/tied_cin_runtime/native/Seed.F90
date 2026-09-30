subroutine tl_cin_native_seed(n,nsn,secondary,ms,iner,saved_mass,saved_inertia,status) bind(C)
  use iso_c_binding
  use ieee_arithmetic
  use cin_native_interfaces, only: max_nodes,max_rows
  implicit none
  integer(c_int), value :: n,nsn
  integer(c_int), intent(in) :: secondary(nsn)
  real(c_double), intent(in) :: ms(n),iner(n)
  real(c_double), intent(inout) :: saved_mass(nsn),saved_inertia(nsn)
  integer(c_int), intent(out) :: status
  type seed_fields
    real(c_double) :: smas(max_rows),siner(max_rows)
  end type
  type(seed_fields) :: intbuf_tab
  real(c_double) :: IN(max_nodes)
  integer, parameter :: IRODDL=1
  integer :: II,I
  status=1
  if(n<1.or.n>max_nodes.or.nsn<1.or.nsn>max_rows) return
  if(any(secondary<1).or.any(secondary>n)) return
  if(.not.all(ieee_is_finite(ms)).or..not.all(ieee_is_finite(iner))) return
  if(any(ms<0).or.any(iner<0)) return
  IN=0
  IN(1:n)=iner
  do II=1,nsn
    I=secondary(II)
#include "Seed.inc"
  end do
  saved_mass=intbuf_tab%smas(1:nsn)
  saved_inertia=intbuf_tab%siner(1:nsn)
  status=0
end subroutine
