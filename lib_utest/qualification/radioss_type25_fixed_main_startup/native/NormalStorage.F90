! SPDX-License-Identifier: AGPL-3.0-or-later
module startup_native_normal_storage
  use iso_c_binding
  implicit none
  real(c_float),allocatable::wnod_normal(:,:,:)
  real(c_float)::observed_rep30=0,observed_rem30=0
  real(c_float)::observed_ready_rep30=0,observed_ready_rem30=0
end module
