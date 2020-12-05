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
include("simulation.jl")

export 
    @dynamics,
    @integrable,
    @direct,
    IntegrableState,
    DirectState,
    DynamicState,
    Dynamics,
    Simulation,
    ComponentCommon,
    LogDataSink,
    simulate,
    create_dynamics,
    log!,
    static

export
    @unpack,
    @pack!

end # module
