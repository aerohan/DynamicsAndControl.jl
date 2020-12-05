using Test
using BenchmarkTools

using DynamicsAndControl
import DynamicsAndControl
using StaticArrays
using OrdinaryDiffEq

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
        g::T
    end
end

function DynamicsAndControl.initialize(::Type{DynamicsFoo}, init_config)
    istate_initial = (a=1.0, b=(@SVector [1.0, 2.0, 3.0]), c=2.0, w=init_config.w0, e=(@MVector [2.0, 5.1, 7.2, 1.4]))
    dstate_initial = (f=false, g=1.2)
    static = init_config

    # returns a tuple of ([initial integrable state], [initial direct state], [configuration parameters])
    return istate_initial, dstate_initial, static
end

function test()
    sim = Simulation( ( :truth, DynamicsFoo, (w0=4.0,) ), 0.0, RK4(), dt=0.1)
    DynamicsAndControl.set_state!(sim.dynamics.x, (a=4.1, b=(@SVector [7.0, 2.0, 3.0]), c=3.2, d=3.0, e=(@MVector [2.0, 5.1, 7.2, 1.4])), (f=true, g=1.4))
    @test sim.dynamics.x.a == 4.1
    @test sim.dynamics.x.b[2] == 2
    @test sim.dynamics.x.e[2] == 5.1
    @test sim.dynamics.x.f == true
    @test sim.dynamics.x.g == 1.4

    DynamicsAndControl.copyto!(sim.dynamics.x_integrable_vector, DynamicsAndControl.integrable_substate(sim.dynamics))
    @test sim.dynamics.x_integrable_vector[1] == 4.1
    @test sim.dynamics.x_integrable_vector[2] == 7.0
    @test sim.dynamics.x_integrable_vector[3] == 2.0
    @test sim.dynamics.x_integrable_vector[6] == 3.0
    @test sim.dynamics.x_integrable_vector[end] == 1.4

    copyto!(sim.dynamics.x_integrable_vector, DynamicsAndControl.integrable_substate(sim.dynamics))

    sim.dynamics.x_integrable_vector[1] *= 1.5
    sim.dynamics.x_integrable_vector[2] *= 1.6
    sim.dynamics.x_integrable_vector[7] *= 1.7

    copyto!(DynamicsAndControl.integrable_substate(sim.dynamics), sim.dynamics.x_integrable_vector)

    @test sim.dynamics.x.a == 4.1*1.5
    @test sim.dynamics.x.b[1] == 7.0*1.6
    @test sim.dynamics.x.e[1] == 2.0*1.7

    @btime copyto!($sim.dynamics.x_integrable_vector, DynamicsAndControl.integrable_substate($sim.dynamics))
    @btime copyto!(DynamicsAndControl.integrable_substate($sim.dynamics), $sim.dynamics.x_integrable_vector)

end

@dynamics DynamicsFoo2{T, N1, N2} begin
    @integrable begin
        a::T
        b::SVector{N1, T}
        c::T
        d::T
        e::MVector{N2, T}
    end
end

function DynamicsAndControl.initialize(::Type{DynamicsFoo2}, init_config)
    istate_initial = (a=2.0, b=(@SVector [1.0, 2.0, 3.0]), c=2.0, w=init_config.w0, e=(@MVector [2.0, 5.1, 7.2, 1.4]))
    dstate_initial = ()
    static = init_config

    # returns a tuple of ([initial integrable state], [initial direct state], [configuration parameters])
    return istate_initial, dstate_initial, static
end

function test2()
    sim = Simulation( ( :truth, DynamicsFoo2, (w0=4.0,) ), 0.0, RK4(), dt=0.1)
    @test sim.dynamics.x.a == 2.0
    @test sim.dynamics.x.b[2] == 2
end

@dynamics DynamicsFoo3 begin
    @integrable begin
        a::Float64
    end
end

function DynamicsAndControl.initialize(::Type{DynamicsFoo3}, init_config)
    istate_initial = (a=1.1)
    dstate_initial = ()
    static = init_config

    # returns a tuple of ([initial integrable state], [initial direct state], [configuration parameters])
    return istate_initial, dstate_initial, static
end

function test3()
    sim = Simulation( ( :truth, DynamicsFoo3, (w0=4.0,) ), 0.0, RK4(), dt=0.1)
    @test sim.dynamics.x.a == 1.1
end

@dynamics DynamicsFoo4 begin
    @direct begin
        a::Float64
    end
end

function DynamicsAndControl.initialize(::Type{DynamicsFoo4}, init_config)
    istate_initial = ()
    dstate_initial = (a=1.1)
    static = init_config

    # returns a tuple of ([initial integrable state], [initial direct state], [configuration parameters])
    return istate_initial, dstate_initial, static
end

function test4()
    sim = Simulation( ( :truth, DynamicsFoo4, (w0=4.0,) ), 0.0, RK4(), dt=0.1)

    @test sim.dynamics.x.a == 1.1
end

test()
test2()
test3()
test4()
