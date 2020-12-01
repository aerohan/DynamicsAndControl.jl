module DynamicsAndControl

using MacroTools
using Infiltrator
using StaticArrays

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

end # module
