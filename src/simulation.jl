struct Simulation{D<:Dynamics, T_t<:Real, T_solver}
    # Components
    dynamics::D
    #sensing::S where S<:Sensing
    #control::C where C<:Control
    
    simtime::SimTime{T_t}
    tspan::Tuple{T_t, T_t}
    dt::T_t
    solver::T_solver

    # Auxiliary
    log_sink::LogDataSink
end

function Simulation(dynamics_args,
                    tspan, solver; dt=error("must specify dt"))
                    #sensing_args,
                    #control_args)

    # set up simulation time span and time object
    tstart, tfinal = process_time_span(tspan)
    simtime = SimTime(tstart)

    log_sink = LogDataSink()

    # initialize the dynamics component. initial state (with concrete types) is
    # returned by the custom initialize call. the static data named tuple is
    # also returned
    #
    # NOTE: initial states should be a tuple with order corresponding to the
    # the fields of the substate struct. they may be written as named tuples
    # for clarity, but the correct order must be maintained, since the tuple is
    # splatted based positional arguments
    dynamics_namespace, T_dyn, config_dyn = dynamics_args
    istate_initial, dstate_initial, static_dyn = initialize(T_dyn, config_dyn)
    component_common = ComponentCommon(simtime, dt, static_dyn, log_sink, dynamics_namespace)
    dynamics = T_dyn(istate_initial, dstate_initial, component_common)

    return Simulation(dynamics, simtime, (tstart, tfinal), dt, solver, log_sink)
end

function simulate(sim::Simulation)
    # set up the initial state
    set_state!(sim.dynamics.x, sim.dynamics.x_initial_integrable, sim.dynamics.x_initial_direct)
    copyto!(sim.dynamics.x_integrable_vector, DynamicsAndControl.integrable_substate(sim.dynamics))
    x0_vector = copy(sim.dynamics.x_integrable_vector)

    # set up the ODE solver
    ode_func! = (ẋ, x, _, t) -> dynamics_ode_interface!(sim, ẋ, x, t)
    ode_problem = ODEProblem(ode_func!, x0_vector, sim.tspan)
    ode_integrator = init(ode_problem, sim.solver, dt=sim.dt, saveat=sim.dt, adaptive=false)

    ode_integrator.t == get(sim.simtime) || error("mismatch between ODE time and sim time")

    # simulation loop
    for integrator_step in ode_integrator
        set!(sim.simtime, integrator_step.t)
    end

    return SimulationDataset(sim.log_sink), ode_integrator.sol
end

function dynamics_ode_interface!(sim, ẋ, x, t)
    copyto!(integrable_substate(sim.dynamics), x)
    compute_ẋ!(sim, t)
    copyto!(ẋ, integrable_substate_derivative(sim.dynamics))
end

function compute_ẋ!(sim::Simulation, t)
    ẋ = integrable_substate_derivative(sim.dynamics)
    x = integrable_substate(sim.dynamics)
    dynamics!(sim.dynamics, ẋ, x, nothing, t)
end

process_time_span(tspan::Tuple) = tspan[1], tspan[2]
process_time_span(tfinal::Real) = zero(typeof(tfinal)), tfinal
