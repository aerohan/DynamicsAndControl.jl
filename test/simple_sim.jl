using Test
using BenchmarkTools
using Revise

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
    @unpack k_spring, mass, c_damping = this.static

    # forces
    f = -k_spring*pos - c_damping*vel

    # derivatives
    ẋ.pos = vel
    ẋ.vel = f/mass
    ẋ.q = @SVector [0.0, 0.0]
end

function ode_dynamics!(ẋ, x, u, t)
    ẋ[1] = x[2]
    ẋ[2] = -u.k_spring * x[1] - u.c_damping * x[2]
end

function test()
    tspan = (0.0, 480.0)

    x0 = (pos=0.5, vel=0.5, q=(@SVector [1.0, 0.0]))
    params = (k_spring=0.6, c_damping=.3, mass=1.0)

    # ODE
    prob = ODEProblem(ode_dynamics!, [x0.pos, x0.vel], tspan, params)
    solve(prob, Tsit5(), dt=0.01, adaptive=false, saveat=0.01)
    @time sol = solve(prob, Tsit5(), dt=0.01, adaptive=false, saveat=0.01)

    # DynamicsAndControl
    sim = Simulation( ( :truth, SimpleTestDynamics, (x0=x0, params=params) ) )
    sol2 = simulate(sim, tspan, 0.01, Tsit5())
    @time sol2 = simulate(sim, tspan, 0.01, Tsit5())

    @test sol.t ≈ sol2.t
    @test sol[1, :] ≈ sol2[1, :]
    @test sol[2, :] ≈ sol2[2, :]
    @test all(sol2[3, :] .≈ 1.0)
    @test all(sol2[4, :] .≈ 0.0)

end

function test_compare_performance()
    x0 = (pos=0.5, vel=0.5, q=(@SVector [1.0, 0.0]))
    params = (k_spring=0.6, c_damping=.3, mass=1.0)
    sim = Simulation( ( :truth, SimpleTestDynamics, (x0=x0, params=params) ) )
    x_vec = similar(sim.dynamics.x_integrable_vector)
    xd_vec = similar(x_vec)
    @btime DynamicsAndControl.dynamics_ode_interface!($sim, $xd_vec, $x_vec, 0.5)

    x_vec = similar([x0.pos, x0.vel])
    xd_vec = similar(x_vec)
    @btime ode_dynamics!($xd_vec, $x_vec, $params,  0.5)
end

test()
test_compare_performance()
