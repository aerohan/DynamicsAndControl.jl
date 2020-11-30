using DynamicsAndControl: @dynamics, @integrable, @direct, IntegrableState, DirectState, DynamicState, Dynamics, Simulation, TelemetrySink
import DynamicsAndControl
using StaticArrays

@dynamics DynamicsFoo{T, N, S} begin
    @integrable begin
        x::Float64
        y::SVector{N, T}
        z::Float64
        w::T
    end

    @direct begin
        m::S
    end
end

function DynamicsAndControl.initialize(::Type{DynamicsFoo}, init_config)
    istate_initial = (x=1.0, y=(@SVector [1, 2, 3]), z=2.0, w=init_config.w0)
    dstate_initial = (z=false,)
    static = init_config

    # returns a tuple of ([initial integrable state], [initial direct state], [configuration parameters])
    return istate_initial, dstate_initial, static
end

sim = Simulation( ( :truth, DynamicsFoo, (w0=4,) ) )

#sim.dynamics.x.m
