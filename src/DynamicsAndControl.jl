module DynamicsAndControl

using MacroTools
using StaticArrays
using OrdinaryDiffEq
using UnPack

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
    DynamicState,
    Simulation,
    Sensor,
    Actuator,
    Controller,
    SensorActuatorControllerState,
    SensorActuatorControllerOutputs,
    ComponentCommon,
    LogDataSink,
    simulate,
    create_dynamics,
    create_sensor_actuator_controller,
    log!,
    static

export
    @unpack,
    @pack!

end # module
