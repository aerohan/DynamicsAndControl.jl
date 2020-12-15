using Test
using BenchmarkTools

using DynamicsAndControl
using StaticArrays
using OrdinaryDiffEq
using Plots

@dynamics SimpleTestDynamics{T} begin
    @integrable begin
        pos::T
        vel::T
        q::SVector{2, T}
    end
end

function DynamicsAndControl.initialize(::Type{SimpleTestDynamics}, config)
    return config.x0, (), config.params
end

function DynamicsAndControl.dynamics!(this::SimpleTestDynamics, ẋ, x, u, t)
    # state
    @unpack pos, vel = x

    # params
    @unpack k_spring, mass, c_damping = static(this)

    # forces
    f = -k_spring*pos - c_damping*vel

    # derivatives
    ẋ.pos = vel
    ẋ.vel = f/mass
    ẋ.q = @SVector [0.0, 0.0]

    log!(this, :state, t, (pos, vel, q=x.q, accel=f/mass))
end

function ode_dynamics!(ẋ, x, u, t)
    ẋ[1] = x[2]
    ẋ[2] = -u.k_spring * x[1] - u.c_damping * x[2]
    ẋ[3] = 0.0
    ẋ[4] = 0.0
end

function test()
    tspan = (0.0, 20.0)

    x0 = (pos=0.5, vel=0.5, q=(@SVector [1.0, 0.0]))
    params = (k_spring=0.6, c_damping=.3, mass=1.0)

    # ODE
    prob = ODEProblem(ode_dynamics!, [x0.pos, x0.vel, x0.q[1], x0.q[2]], tspan, params)
    sol = solve(prob, Tsit5(), dt=0.1, adaptive=false, saveat=0.1)

    # DynamicsAndControl
    sim = Simulation( ( :truth, SimpleTestDynamics, (x0=x0, params=params) ), tspan, Tsit5(), dt=0.1)
    data = simulate(sim)

    state = data.truth.state

    @test all(sol.t .≈ state.time)
    @test all(sol[1, :] .≈ state.pos)
    @test all(sol[2, :] .≈ state.vel)
    @test all(state.q[1] .≈ 1.0)
    @test all(state.q[2] .≈ 0.0)

end

function test_compare_performance()
    x0 = (pos=0.5, vel=0.5, q=(@SVector [1.0, 0.0]))
    params = (k_spring=0.6, c_damping=.3, mass=1.0)
    sim = Simulation( ( :truth, SimpleTestDynamics, (x0=x0, params=params) ), (0.0, 10.0), Tsit5(), dt=0.1)
    x_vec = similar(DynamicsAndControl.integrable_vector(sim.dynamics))
    xd_vec = similar(x_vec)

    bench1 = @benchmark DynamicsAndControl.dynamics_ode_interface!($sim, $xd_vec, $x_vec, 0.5)
    @test bench1.allocs == 0

    x_vec = similar([x0.pos, x0.vel, x0.q[1], x0.q[2]])
    xd_vec = similar(x_vec)
    bench2 = @benchmark ode_dynamics!($xd_vec, $x_vec, $params,  0.5)
    @test bench2.allocs == 0
end

test()
test_compare_performance()
