using Test
using BenchmarkTools
using Revise

using DynamicsAndControl: @dynamics, @integrable, @direct, IntegrableState, DirectState, DynamicState, Dynamics, Simulation, TelemetrySink
using DynamicsAndControl: simulate, create_dynamics
import DynamicsAndControl
using StaticArrays

@dynamics DynamicsFoo{T, N1, N2, S} begin
    @integrable begin
        a::T
        b::SVector{N1, T}
        c::T
        d::T
        e::MVector{N2, T}
    end

    @direct begin
        f::S
    end
end

function DynamicsAndControl.initialize(::Type{DynamicsFoo}, init_config)
    istate_initial = (a=1.0, b=(@SVector [1.0, 2.0, 3.0]), c=2.0, w=init_config.w0, e=(@MVector [2.0, 5.1, 7.2, 1.4]))
    dstate_initial = (f=false,)
    static = init_config

    # returns a tuple of ([initial integrable state], [initial direct state], [configuration parameters])
    return istate_initial, dstate_initial, static
end

function test()
    sim = Simulation( ( :truth, DynamicsFoo, (w0=4.0,) ) )
    DynamicsAndControl.set_state!(sim.dynamics.x, (a=4.1, b=(@SVector [7.0, 2.0, 3.0]), c=3.2, d=3.0, e=(@MVector [2.0, 5.1, 7.2, 1.4])), (f=true,))
    @test sim.dynamics.x.a == 4.1
    @test sim.dynamics.x.b[2] == 2
    @test sim.dynamics.x.e[2] == 5.1
    @test sim.dynamics.x.f == true

    DynamicsAndControl.copyto!(sim.dynamics.x_integrable_vector, DynamicsAndControl.integrable_substate(sim.dynamics))
    @test sim.dynamics.x_integrable_vector[1] == 4.1
    @test sim.dynamics.x_integrable_vector[2] == 7.0
    @test sim.dynamics.x_integrable_vector[3] == 2.0
    @test sim.dynamics.x_integrable_vector[6] == 3.0
    @test sim.dynamics.x_integrable_vector[end] == 1.4

    sim = simulate(sim, (0.0, 10.0))

    @test sim.dynamics.x.a == 1.0*1.5
    @test sim.dynamics.x.b[1] == 1.0*1.6
    @test sim.dynamics.x.e[1] == 2.0*1.7

    @btime simulate($sim, (0.0, 10.0))
end

test()
