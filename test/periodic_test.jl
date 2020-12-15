using Test
using DynamicsAndControl
using OrdinaryDiffEq

function test_real()
    sim_dt = 0.1
    ode_problem = ODEProblem((_,_,_)->[1.0], [0.0], (0.0, 10.0))
    ode_integrator = init(ode_problem, RK4(), dt=sim_dt, saveat=sim_dt, adaptive=false)

    p = PeriodicReal(.5, eps=eps(1.0))

    d = Float64[]
    dispatch(p, ode_integrator.t) do
        push!(d, ode_integrator.t)
    end

    for integrator_step in ode_integrator
        dispatch(p, integrator_step.t) do
            push!(d, ode_integrator.t)
        end
    end

    @test all(abs.(diff(d) .- 0.5) .< sim_dt)

end

function test_fixed()
    sim_dt = 0.1
    ode_problem = ODEProblem((_,_,_)->[1.0], [0.0], (0.0, 10.0))
    ode_integrator = init(ode_problem, RK4(), dt=sim_dt, saveat=sim_dt, adaptive=false)

    p = PeriodicFixed(.5)

    d = Float64[]
    dispatch(p, ode_integrator.t, sim_dt) do
        push!(d, ode_integrator.t)
    end

    for integrator_step in ode_integrator
        dispatch(p, integrator_step.t, sim_dt) do
            push!(d, ode_integrator.t)
        end
    end

    @test all(diff(d) .≈ 0.5)
end

test_real()
test_fixed()
