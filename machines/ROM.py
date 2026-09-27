import numpy as np
from physics.reduced_order import solve_axial_rotor, solve_axial_stator, solve_radial_rotor, solve_radial_stator, solve_CV, get_dE_dt, get_dm_dt, get_power, get_torque, get_domega_dt
from models.simple import BladeParams,IdealGas, SystemState, ControlVolume, Derivatives, MachineEvaluation

def evaluate_axial_compressor(state1: SystemState,
                state2: SystemState,
                blade: BladeParams, 
                fluid: IdealGas,
                inlet: ControlVolume) -> Derivatives:

    station1 = solve_CV(state1, fluid, inlet)

    rotor = solve_axial_rotor(station1.outlet, blade, fluid, station1)

    station2 = solve_CV(state2, fluid, rotor.params)

    stator = solve_axial_stator(station2.outlet, blade, station2)

    dm_dt1 = get_dm_dt(station1.m_dot_in, station1.m_dot_out)
    dE_dt1 = get_dE_dt(station1.m_dot_in, station1.m_dot_out, station1.h0_in, station1.h0_out, station1.W_s_dot, station1.Q_dot)

    dm_dt2 = get_dm_dt(station2.m_dot_in, station2.m_dot_out)
    dE_dt2 = get_dE_dt(station2.m_dot_in, station2.m_dot_out, station2.h0_in, station2.h0_out, station2.W_s_dot, station2.Q_dot)    

    power = get_power(rotor.delta_h0, rotor.outlet.m_dot)
    torque = get_torque(power, rotor.state.omega)

    domega_dt = get_domega_dt(torque, blade.tau_load, blade.I)

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



def evaluate_axial_turbine(state1: SystemState,
                state2: SystemState,
                blade: BladeParams, 
                fluid: IdealGas,
                inlet: ControlVolume) -> Derivatives:

    station1 = solve_CV(state1, fluid, inlet)

    stator = solve_axial_stator(station1.outlet, blade, station1)

    station2 = solve_CV(state2, fluid, stator.params)

    rotor = solve_axial_rotor(station2.outlet, blade, fluid, station2)

    dm_dt1 = get_dm_dt(station1.m_dot_in, station1.m_dot_out)
    dE_dt1 = get_dE_dt(station1.m_dot_in, station1.m_dot_out, station1.h0_in, station1.h0_out, station1.W_s_dot, station1.Q_dot)

    dm_dt2 = get_dm_dt(station2.m_dot_in, station2.m_dot_out)
    dE_dt2 = get_dE_dt(station2.m_dot_in, station2.m_dot_out, station2.h0_in, station2.h0_out, station2.W_s_dot, station2.Q_dot)    

    power = get_power(rotor.delta_h0, rotor.outlet.m_dot)
    torque = get_torque(power, rotor.state.omega)

    domega_dt = get_domega_dt(torque, blade.tau_load, blade.I)

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

def evaluate_radial_compressor(state1: SystemState,
                state2: SystemState,
                blade: BladeParams, 
                fluid: IdealGas,
                inlet: ControlVolume) -> Derivatives:

    station1 = solve_CV(state1, fluid, inlet)

    rotor = solve_radial_rotor(station1.outlet, blade, fluid, station1)

    station2 = solve_CV(state2, fluid, rotor.params)

    stator = solve_radial_stator(station2.outlet, blade, station2)

    dm_dt1 = get_dm_dt(station1.m_dot_in, station1.m_dot_out)
    dE_dt1 = get_dE_dt(station1.m_dot_in, station1.m_dot_out, station1.h0_in, station1.h0_out, station1.W_s_dot, station1.Q_dot)

    dm_dt2 = get_dm_dt(station2.m_dot_in, station2.m_dot_out)
    dE_dt2 = get_dE_dt(station2.m_dot_in, station2.m_dot_out, station2.h0_in, station2.h0_out, station2.W_s_dot, station2.Q_dot)    

    power = get_power(rotor.delta_h0, rotor.outlet.m_dot)
    torque = get_torque(power, rotor.state.omega)

    domega_dt = get_domega_dt(torque, blade.tau_load, blade.I)

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


def evaluate_radial_turbine(state1: SystemState,
                state2: SystemState,
                blade: BladeParams, 
                fluid: IdealGas,
                inlet: ControlVolume) -> Derivatives:

    station1 = solve_CV(state1, fluid, inlet)

    stator = solve_radial_stator(station1.outlet, blade, station1)

    station2 = solve_CV(state2, fluid, stator.params)

    rotor = solve_radial_rotor(station2.outlet, blade, fluid, station2)

    dm_dt1 = get_dm_dt(station1.m_dot_in, station1.m_dot_out)
    dE_dt1 = get_dE_dt(station1.m_dot_in, station1.m_dot_out, station1.h0_in, station1.h0_out, station1.W_s_dot, station1.Q_dot)

    dm_dt2 = get_dm_dt(station2.m_dot_in, station2.m_dot_out)
    dE_dt2 = get_dE_dt(station2.m_dot_in, station2.m_dot_out, station2.h0_in, station2.h0_out, station2.W_s_dot, station2.Q_dot)    

    power = get_power(rotor.delta_h0, rotor.outlet.m_dot)
    torque = get_torque(power, rotor.state.omega)

    domega_dt = get_domega_dt(torque, blade.tau_load, blade.I)

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