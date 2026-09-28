from physics.reduced_order import solve_axial_rotor, solve_axial_stator, solve_radial_rotor, solve_radial_stator, solve_CV, get_dE_dt, get_dm_dt, get_power, get_torque, get_domega_dt
from models.simple import BladeParams,IdealGas, SystemState, ControlVolume, Derivatives, MachineEvaluation, MachineType

# Evaluate the state derivatives of an axial compressor for one time step.
#
# Order: CV1 -> Rotor -> CV2 -> Stator 
#
# Update 5 states: 
#   m1, 
#   E1,
#   m2,
#   E2,
#   omega
#
# Output state derivatives with outlet thermodynamic states
def evaluate_axial_compressor(state1: SystemState,
                state2: SystemState,
                blade: BladeParams, 
                fluid: IdealGas,
                inlet: ControlVolume,
                machine_type: MachineType) -> Derivatives:

    # Find density and thermodynamic states of fluid at CV1
    station1 = solve_CV(inlet.outlet, state1, fluid, inlet)

    # Find enthalpy change and stagnation states across rotor
    rotor = solve_axial_rotor(station1.outlet, state1, blade, fluid, station1, machine_type)

    # Find density and thermodynamic states of fluid at CV2    
    station2 = solve_CV(rotor.outlet, state2, fluid, rotor.params)

    # Find c_theta changes across stator
    stator = solve_axial_stator(station2.outlet, blade, station2)

    # Get the derivatives of m1 and E1 through ODE's
    dm_dt1 = get_dm_dt(station1.m_dot_in, station1.m_dot_out)
    dE_dt1 = get_dE_dt(station1.m_dot_in, station1.m_dot_out, station1.h0_in, station1.h0_out, station1.W_s_dot, station1.Q_dot)

    # Get the derivatives of m2 and E2 through ODE's
    dm_dt2 = get_dm_dt(station2.m_dot_in, station2.m_dot_out)
    dE_dt2 = get_dE_dt(station2.m_dot_in, station2.m_dot_out, station2.h0_in, station2.h0_out, station2.W_s_dot, station2.Q_dot)    

    # Calculate power and torque from the rotor output
    power = get_power(rotor.delta_h0, rotor.params.m_dot_out)
    torque = get_torque(power, rotor.state.omega)

    # Get derivative of omega from ODE
    domega_dt = get_domega_dt(torque, blade.tau_load, blade.I)

    # Output MachineEvaluation object with derivatives, thermodynamic and velocity states
    return MachineEvaluation(
        outlet=stator.outlet,
        params=stator.params,
        dervatives= Derivatives(
        dm_dt1=dm_dt1,
        dm_dt2=dm_dt2,
        dE_dt1=dE_dt1,
        dE_dt2=dE_dt2,
        domega_dt=domega_dt
        )
    )


# Evaluate the state derivatives of an axial turbine for one time step.
#
# Order: CV1 -> Stator -> CV2 -> Rotor 
#
# Update 5 states: 
#   m1, 
#   E1,
#   m2,
#   E2,
#   omega
#
# Output state derivatives with outlet thermodynamic states
def evaluate_axial_turbine(state1: SystemState,
                state2: SystemState,
                blade: BladeParams, 
                fluid: IdealGas,
                inlet: ControlVolume,
                machine_type: MachineType) -> Derivatives:

    # Find density and thermodynamic states of fluid at CV1
    station1 = solve_CV(inlet.outlet, state1, fluid, inlet)

    # Find c_theta changes across stator
    stator = solve_axial_stator(station1.outlet, blade, station1)

    # Find density and thermodynamic states of fluid at CV2
    station2 = solve_CV(stator.outlet, state2, fluid, stator.params)

    # Find enthalpy change and stagnation states across rotor
    rotor = solve_axial_rotor(station2.outlet, state1, blade, fluid, station2, machine_type)

    # Get the derivatives of m1 and E1 through ODE's
    dm_dt1 = get_dm_dt(station1.m_dot_in, station1.m_dot_out)
    dE_dt1 = get_dE_dt(station1.m_dot_in, station1.m_dot_out, station1.h0_in, station1.h0_out, station1.W_s_dot, station1.Q_dot)

    # Get the derivatives of m2 and E2 through ODE's
    dm_dt2 = get_dm_dt(station2.m_dot_in, station2.m_dot_out)
    dE_dt2 = get_dE_dt(station2.m_dot_in, station2.m_dot_out, station2.h0_in, station2.h0_out, station2.W_s_dot, station2.Q_dot)    

    # Calculate power and torque from the rotor output
    power = get_power(rotor.delta_h0, rotor.outlet.m_dot)
    torque = get_torque(power, rotor.state.omega)

    # Get derivative of omega from ODE
    domega_dt = get_domega_dt(torque, blade.tau_load, blade.I)

    # Output MachineEvaluation object with derivatives, thermodynamic and velocity states
    return MachineEvaluation(
        outlet=rotor.outlet,
        params=rotor.params,
        dervatives= Derivatives(
        dm_dt1=dm_dt1,
        dm_dt2=dm_dt2,
        dE_dt1=dE_dt1,
        dE_dt2=dE_dt2,
        domega_dt=domega_dt
        )
    )

# Evaluate the state derivatives of an radial compressor for one time step.
#
# Order: CV1 -> Rotor -> CV2 -> Stator 
#
# Update 5 states: 
#   m1, 
#   E1,
#   m2,
#   E2,
#   omega
#
# Output state derivatives with outlet thermodynamic states
def evaluate_radial_compressor(state1: SystemState,
                state2: SystemState,
                blade: BladeParams, 
                fluid: IdealGas,
                inlet: ControlVolume,
                machine_type: MachineType) -> Derivatives:

    # Find density and thermodynamic states of fluid at CV1
    station1 = solve_CV(inlet.outlet, state1, fluid, inlet)

    # Find enthalpy change and stagnation states across rotor
    rotor = solve_radial_rotor(station1.outlet, state1, blade, fluid, station1, machine_type)

    # Find density and thermodynamic states of fluid at CV2
    station2 = solve_CV(rotor.outlet, state2, fluid, rotor.params)

    # Find c_theta changes across stator
    stator = solve_radial_stator(station2.outlet, blade, station2)

    # Get the derivatives of m1 and E1 through ODE's
    dm_dt1 = get_dm_dt(station1.m_dot_in, station1.m_dot_out)
    dE_dt1 = get_dE_dt(station1.m_dot_in, station1.m_dot_out, station1.h0_in, station1.h0_out, station1.W_s_dot, station1.Q_dot)

    # Get the derivatives of m2 and E2 through ODE's
    dm_dt2 = get_dm_dt(station2.m_dot_in, station2.m_dot_out)
    dE_dt2 = get_dE_dt(station2.m_dot_in, station2.m_dot_out, station2.h0_in, station2.h0_out, station2.W_s_dot, station2.Q_dot)    

    # Calculate power and torque from the rotor output
    power = get_power(rotor.delta_h0, rotor.outlet.m_dot)
    torque = get_torque(power, rotor.state.omega)

    # Get derivative of omega from ODE
    domega_dt = get_domega_dt(torque, blade.tau_load, blade.I)

    # Output MachineEvaluation object with derivatives, thermodynamic and velocity states
    return MachineEvaluation(
        outlet=stator.outlet,
        params=stator.params,
        dervatives= Derivatives(
        dm_dt1=dm_dt1,
        dm_dt2=dm_dt2,
        dE_dt1=dE_dt1,
        dE_dt2=dE_dt2,
        domega_dt=domega_dt
        )
    )

# Evaluate the state derivatives of an radial turbine for one time step.
#
# Order: CV1 -> Stator -> CV2 -> Rotor 
#
# Update 5 states: 
#   m1, 
#   E1,
#   m2,
#   E2,
#   omega
#
# Output state derivatives with outlet thermodynamic states
def evaluate_radial_turbine(state1: SystemState,
                state2: SystemState,
                blade: BladeParams, 
                fluid: IdealGas,
                inlet: ControlVolume,
                machine_type: MachineType) -> Derivatives:

    # Find density and thermodynamic states of fluid at CV1
    station1 = solve_CV(inlet.outlet, state1, fluid, inlet)

    # Find c_theta changes across stator
    stator = solve_radial_stator(station1.outlet, blade, station1)

    # Find density and thermodynamic states of fluid at CV2
    station2 = solve_CV(stator.outlet, state2, fluid, stator.params)

    # Find enthalpy change and stagnation states across rotor
    rotor = solve_radial_rotor(station2.outlet, state1, blade, fluid, station2, machine_type)

    # Get the derivatives of m1 and E1 through ODE's
    dm_dt1 = get_dm_dt(station1.m_dot_in, station1.m_dot_out)
    dE_dt1 = get_dE_dt(station1.m_dot_in, station1.m_dot_out, station1.h0_in, station1.h0_out, station1.W_s_dot, station1.Q_dot)

    # Get the derivatives of m2 and E2 through ODE's
    dm_dt2 = get_dm_dt(station2.m_dot_in, station2.m_dot_out)
    dE_dt2 = get_dE_dt(station2.m_dot_in, station2.m_dot_out, station2.h0_in, station2.h0_out, station2.W_s_dot, station2.Q_dot)    

    # Calculate power and torque from the rotor output
    power = get_power(rotor.delta_h0, rotor.outlet.m_dot)
    torque = get_torque(power, rotor.state.omega)

    # Get derivative of omega from ODE
    domega_dt = get_domega_dt(torque, blade.tau_load, blade.I)

    # Output MachineEvaluation object with derivatives, thermodynamic and velocity states
    return MachineEvaluation(
        outlet=rotor.outlet,
        params=rotor.params,
        dervatives= Derivatives(
        dm_dt1=dm_dt1,
        dm_dt2=dm_dt2,
        dE_dt1=dE_dt1,
        dE_dt2=dE_dt2,
        domega_dt=domega_dt
        )
    )