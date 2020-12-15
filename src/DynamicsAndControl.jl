module DynamicsAndControl

using MacroTools
using StaticArrays
using OrdinaryDiffEq
using UnPack
using RecipesBase
using Rotations
using MeshCat

# debugging only
using Infiltrator


include("types.jl")
include("telemetry.jl")
include("components.jl")
include("builtin_components.jl")
include("simulation.jl")

export 
    @dynamics,
    @integrable,
    @direct,
    @sensor,
    @actuator,
    @controller,
    @state,
    @outputs,
    Dynamics,
    IntegrableState,
    DirectState,
    Simulation,
    Sensor,
    Actuator,
    Controller,
    SensorActuatorControllerState,
    SensorActuatorControllerOutputs,
    ComponentData,
    DynamicsData,
    SensorActuatorControllerData,
    simulate,
    LogDataSink,
    PeriodicReal,
    PeriodicFixed,
    dispatch,
    create_dynamics,
    create_sensor_actuator_controller,
    log!,
    static,
    series

export
    @unpack,
    @pack!,
    RK4

end # module
