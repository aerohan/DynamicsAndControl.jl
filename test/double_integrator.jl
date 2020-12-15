using DynamicsAndControl
using StaticArrays
import OrdinaryDiffEq
using BenchmarkTools
using Test

@dynamics DoubleIntegrator{T, I} begin
    @integrable begin
        r::SVector{2, T}
        v::SVector{2, T}
        big::MVector{10, T}
    end

    @direct begin
        y::I
    end
end

DynamicsAndControl.initialize(::Type{DoubleIntegrator}, config) = (@SVector([3.0, 2.0]), @SVector([2.0, -3.0]), MVector{10}(rand(10))), (1,), NamedTuple()

function DynamicsAndControl.dynamics!(this::DoubleIntegrator, ẋ, x, u, t)
    ẋ.r = x.v
    ẋ.v = u.a
    ẋ.big = -x.big

    log!(this, (), t, (r=ẋ.r, v=ẋ.v, big=ẋ.big))
end

@sensor PositionSensor{T} begin
    @outputs begin
        r_sense::SVector{2, T}
    end
end

DynamicsAndControl.initialize(::Type{PositionSensor}, config) = (), (@SVector([0.0, 0.0]),), NamedTuple()

function DynamicsAndControl.update!(this::PositionSensor, y, state, x, t)
    y.r_sense = 1.1 * x.r
end

@controller DoubleIntegratorController{T} begin
    @outputs begin
        a::SVector{2, T}
    end
end

DynamicsAndControl.initialize(::Type{DoubleIntegratorController}, config) = (), (@SVector([0.0, 0.0]),), NamedTuple()

function DynamicsAndControl.update!(this::DoubleIntegratorController, u, state, y, t)
    u.a = -3.2 * y.r_sense

    log!(this, (), t, (a=u.a,))
end

struct FakeOdeIntegrator
    t::Float64
end
OrdinaryDiffEq.set_u!(f::FakeOdeIntegrator, args...) = nothing
OrdinaryDiffEq.u_modified!(f::FakeOdeIntegrator, args...) = nothing

function _step(sim, integrator, x0, t)
    DynamicsAndControl.set!(sim.simtime, t)
    DynamicsAndControl.copyto!(DynamicsAndControl.integrable_substate(sim.dynamics), x0)
    DynamicsAndControl.update_sensor_controller_actuator!(sim)
    DynamicsAndControl.update_dynamics!(sim, integrator)
end

function test()
    sim = Simulation( 
                         ( :truth, DoubleIntegrator, () ), 
                         ( :sensor, PositionSensor, () ), 
                         ( :controller, DoubleIntegratorController, () ), 
                         20.0, RK4(), dt=0.2
                    )
    data = simulate(sim)

    t = 1.5
    fake_integrator = FakeOdeIntegrator(t)
    x0 = copy(DynamicsAndControl.integrable_vector(sim.dynamics))
    bench = @benchmark _step($sim, $fake_integrator, $x0, $t)
    @test bench.allocs == 0
end

test()
