# Iteratively solve the thermodynamic and velocity state at a single
# turbomachinery station.
#
# Known quantities:
#   m_dot   = mass flow rate [kg/s]
#   A       = flow area [m^2]
#   c_theta = tangential/whirl component of absolute velocity [m/s]
#   T0      = stagnation temperature [K]
#   P0      = stagnation pressure [Pa]
#   gamma   = ratio of specific heats [-]
#   R       = specific gas constant [J/(kg K)]
#
# Other inputs:
# eps      = density convergence tolerance
# max_iter = maximum permitted number of density iterations
#
# Returns a FlowState containing the converged static, stagnation,
# and velocity properties at the station.
def solve_station(inlet: FlowState, blade: BladeParams, fluid: IdealGas, eps = 1e-12, max_iter = 1000):

    # Inlet variables
    m_dot = inlet.m_dot
    c_theta = inlet.c_theta
    T0 = inlet.T0
    P0 = inlet.P0

    # Blade variables
    A = blade.A

    # Fluid variables
    gamma = fluid.gamma
    R = fluid.R

    # Calculate the constant-pressure specific heat for a calorically
    # perfect gas:
    c_p = gamma * R / (gamma - 1)

    # Initial guess for density [kg/m^3].
    rho = 1

    for n in range(max_iter):

        # Calculate meridional velocity using conservation of mass:
        c_x = m_dot / (rho * A)

        # Calculate the magnitude of the absolute velocity from its
        # meridional and tangential components:
        c = np.sqrt(c_x**2 + c_theta**2)

        # Convert stagnation temperature to static temperature using
        # the steady-flow energy relationship:
        T = T0 - c**2/(2*c_p)

        # Calculate the local speed of sound using the static
        # temperature:
        a = np.sqrt(gamma * R * T)

        # Calculate the absolute Mach number:
        M = c/a

        # Calculate static pressure from stagnation pressure using the
        # isentropic perfect-gas stagnation/static pressure relation:
        P = P0 / ((1 + M**2 * (gamma - 1)/2) ** (gamma/(gamma - 1)))

        # Calculate a new density from the ideal-gas equation of state:
        new_rho = P / (R * T)

        # Check whether the density has converged.
        if abs(rho - new_rho) < eps:
            rho = new_rho
            break

        rho = new_rho

    # Error message if exceed max iterations
    else:
        raise RuntimeError("Density iteration did not converge")

    # Package the converged station properties into a FlowState object.
    return FlowState(
        P=P,
        T=T,
        rho=rho,
        P0=P0,
        T0=T0,
        c_x=c_x,
        c_theta=c_theta,
        M=M
    )


def forward_euler(inlet: FlowState, blade: BladeParams, fluid: IdealGas, time_params: TimeParams, machine_type, flow_geometry):

    n_steps = time_params.n_steps

    for n in range(n_steps):

        station_1 = solve_station(inlet, blade, fluid)

        if machine_type == MachineType.COMPRESSOR:

            if flow_geometry == FlowGeometry.AXIAL:

                compressor_rotor = solve_axial_rotor(station_1, blade, fluid)

                omega = compressor_rotor.omega
                delta_h0 = compressor_rotor.delta_h0

                station_2 = solve_station(compressor_rotor.outlet, blade, fluid)

                compressor_stator = solve_axial_stator(station_2, blade)

                station_3 = solve_station(compressor_stator.outlet, blade, fluid)

            elif flow_geometry == FlowGeometry.RADIAL:

                compressor_rotor = solve_radial_rotor(station_1, blade, fluid)

                omega = compressor_rotor.omega
                delta_h0 = compressor_rotor.delta_h0

                station_2 = solve_station(compressor_rotor.outlet, blade, fluid)

                compressor_stator = solve_radial_stator(station_2, blade)

                station_3 = solve_station(compressor_stator.outlet, blade, fluid)

            else:
                raise RuntimeError("Flow type not defined")

        elif machine_type == MachineType.TURBINE:

            if flow_geometry == FlowGeometry.AXIAL:

                turbine_stator = solve_axial_stator(station_1, blade)

                station_2 = solve_station(turbine_stator.outlet, blade, fluid)

                turbine_rotor = solve_axial_rotor(station_2, blade, fluid)

                omega = turbine_rotor.omega
                delta_h0 = turbine_rotor.delta_h0

                station_3 = solve_station(turbine_rotor.outlet, blade, fluid)

            elif flow_geometry == FlowGeometry.RADIAL:

                turbine_stator = solve_radial_stator(station_1, blade)            

                station_2 = solve_station(turbine_rotor.outlet, blade, fluid)

                turbine_rotor = solve_radial_rotor(station_2, blade, fluid)

                omega = turbine_rotor.omega
                delta_h0 = turbine_rotor.delta_h0

                station_3 = solve_station(turbine_stator.outlet, blade, fluid)

            else:
                raise RuntimeError("Flow type not defined")

        else:
            raise RuntimeError("Machine type not defined")

        m_dot = inlet.m_dot

        power = get_power(delta_h0, m_dot)
        torque = get_torque(power, omega)

        tau_load = blade.tau_load
        I = blade.I

        delta_t = time_params.delta_t

        domega_dt = (torque - tau_load) / I

        omega_new = omega + delta_t * domega_dt

        omega = omega_new

    return SystemState(
        outlet=station_3,
        omega=omega
    )
