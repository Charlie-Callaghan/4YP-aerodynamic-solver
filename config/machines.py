from models.simple import MachineType, FlowGeometry, Machine

axial_compressor = Machine(
    machine_type=MachineType.COMPRESSOR,
    flow_geometry=FlowGeometry.AXIAL
)

radial_compressor = Machine(
    machine_type=MachineType.COMPRESSOR,
    flow_geometry=FlowGeometry.RADIAL
)

axial_turbine = Machine(
    machine_type=MachineType.TURBINE,
    flow_geometry=FlowGeometry.AXIAL
)

radial_turbine = Machine(
    machine_type=MachineType.TURBINE,
    flow_geometry=FlowGeometry.RADIAL
)

francis_turbine = Machine(
    machine_type=MachineType.TURBINE,
    flow_geometry=FlowGeometry.MIXED
)

