using DynamicsAndControl
using StaticArrays
using OrdinaryDiffEq
using Rotations

@dynamics QuatTestDynamics{T} begin
    @integrable begin
        q::Quat{T}
    end

    @direct begin
        b::Bool
    end
end

DynamicsAndControl.initialize(::Type{QuatTestDynamics}, config) = (Quat(1.0, 0.0, 0.0, 0.0),), (true,), NamedTuple()

function DynamicsAndControl.dynamics!(this::QuatTestDynamics, ẋ, x, u, t)
    ẋ.q = @SVector [0.0, 1.0, 0.0, 0.0]

    log!(this, (), t, (q=x.q, b=x.b))
end

function test()
    sim = Simulation((:truth, QuatTestDynamics, ()), (0.0, 10.0), RK4(), dt=.1)
    data = simulate(sim)
end

