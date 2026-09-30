module T45_CONSTRUCTOR_PACKET
  use iso_c_binding
  use T45_STARTUP_PACKET, only: StartupPhase
  use T45_STEP_PACKET, only: StepPhase
  implicit none
contains
  subroutine type45_native_constructor(kind,property,xinput,roles,damping,coefficient,spin, &
      uvar,history,observation,status) bind(C)
    integer(c_int), intent(in) :: kind,roles(2)
    real(c_double), intent(in) :: property(14),xinput(3,5),damping(2,2),coefficient(4,2),spin(3,2)
    real(c_double), intent(inout) :: uvar(39),history(13),observation(25)
    integer(c_int), intent(out) :: status
    real(c_double) :: var(39),h(13),initial(13),step(25)
    status=1
    ! Named selected profile: original optional free K/C are all absent/zero.
    if(any(property(3:14)/=0)) return
    var=0
    h=0
    initial=0
    step=0
    ! No selected future timestep or automatic stiffness exists in this phase.
    call StartupPhase(kind,property,xinput,roles,damping,coefficient,0.d0,.false.,var,initial,status)
    if(status/=0) return
    call StepPhase(kind,property,xinput(:,1:2),spin,0.d0,0.d0,0,.true.,var,h,step,status)
    if(status/=0) return
    uvar=var
    history=h
    observation=step
  end subroutine
end module
