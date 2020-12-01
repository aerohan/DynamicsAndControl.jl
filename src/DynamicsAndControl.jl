module DynamicsAndControl

using MacroTools
using StaticArrays
using OrdinaryDiffEq
using UnPack

# debugging only
using Infiltrator


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
    TelemetrySink,
    simulate,
    create_dynamics

export
    @unpack,
    @pack!

end # module
