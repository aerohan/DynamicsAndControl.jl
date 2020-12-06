struct Simulation{D<:Dynamics,S<:Sensor,C<:Controller,A<:Actuator,T_t<:Real,T_solver}
    # Components
    dynamics::D
    sensor::S
    controller::C
    actuator::A
    
    simtime::SimTime{T_t}
    tspan::Tuple{T_t, T_t}
    dt::T_t
    solver::T_solver

    # Auxiliary
    log_sink::LogDataSink
end

function Simulation(dynamics_args,
                    sensor_args,
                    controller_args,
                    actuator_args,
                    tspan, solver; dt=error("must specify dt"))

    # set up simulation time span and time object
    tstart, tfinal = process_time_span(tspan)
    simtime = SimTime(tstart)

    log_sink = LogDataSink()

    # initialize each of the components. initial states/outputs (with concrete
    # types) is returned by the custom initialize call. the static data named
    # tuple is also returned
    #
    # NOTE: initial states/outputs should be a tuple with order corresponding
    # to the the fields of the subcomponent struct. they may be written as
    # named tuples for clarity, but the correct order must be maintained, since
    # the tuple is splatted based positional arguments
    dynamics = _setup(Dynamics, dynamics_args, simtime, dt, log_sink)
    sensor = _setup(Sensor, sensor_args, simtime, dt, log_sink, dynamics)
    controller = _setup(Sensor, controller_args, simtime, dt, log_sink, sensor)
    actuator = _setup(Actuator, actuator_args, simtime, dt, log_sink, controller)

    return Simulation(dynamics, sensor, controller, actuator, simtime, (tstart, tfinal), dt, solver, log_sink)
end

function _setup(::Type{Dynamics}, dynamics_args, simtime, dt, log_sink)
    namespace, T_dyn, config_dyn = dynamics_args
    istate_initial, dstate_initial, static_dyn = initialize(T_dyn, config_dyn)
    component_common = ComponentCommon(simtime, dt, dt, static_dyn, log_sink, namespace)
    dynamics = T_dyn(istate_initial, dstate_initial, component_common)

    return dynamics
end

# TODO: 3 methods below can probably be combined into 2
function _setup(::Type{Sensor}, sensor_args, simtime, dt, log_sink, dynamics)
    namespace, T_s, config_s = sensor_args
    state_initial, outputs_initial, static_s = initialize(T_s, config_s)
    component_common = ComponentCommon(simtime, dt, dt, static_s, log_sink, namespace)
    sensor = T_s(state_initial, outputs_initial, component_common, dynamics)

    return sensor
end

function _setup(::Type{Controller}, controller_args, simtime, dt, log_sink, sensor)
    namespace, T_c, config_c = controller_args
    state_initial, outputs_initial, static_c = initialize(T_c, config_c)
    component_common = ComponentCommon(simtime, dt, dt, static_c, log_sink, namespace)
    controller = T_c(state_initial, outputs_initial, component_common, sensor)

    return controller
end

function _setup(::Type{Actuator}, actuator_args, simtime, dt, log_sink, controller)
    namespace, T_a, config_a = actuator_args
    state_initial, outputs_initial, static_a = initialize(T_a, config_a)
    component_common = ComponentCommon(simtime, dt, dt, static_a, log_sink, namespace)
    actuator = T_a(state_initial, outputs_initial, component_common, controller)

    return actuator
end

function simulate(sim::Simulation)
    # set up the initial state
    set_state!(sim.dynamics.x, sim.dynamics.x_initial_integrable, sim.dynamics.x_initial_direct)
    copyto!(sim.dynamics.x_integrable_vector, DynamicsAndControl.integrable_substate(sim.dynamics))
    x0_vector = copy(sim.dynamics.x_integrable_vector)

    # generate the controls at the first timestep
    update_sensor_controller_actuator!(sim)

    # set up the ODE solver
    ode_func! = (ẋ, x, _, t) -> dynamics_ode_interface!(sim, ẋ, x, t)
    ode_problem = ODEProblem(ode_func!, x0_vector, sim.tspan)
    ode_integrator = init(ode_problem, sim.solver, dt=sim.dt, saveat=sim.dt, adaptive=false)

    # update the dynamics for the start of the first integration step
    update_dynamics!(sim, ode_integrator)

    ode_integrator.t == get(sim.simtime) || error("mismatch between ODE time and sim time")

    # simulation loop
    for integrator_step in ode_integrator
        # update the simtime
        set!(sim.simtime, integrator_step.t)

        # copy the latest integrated state
        copyto!(integrable_substate(sim.dynamics), integrator_step.u)

        # generate new controls
        update_sensor_controller_actuator!(sim)

        # update the dynamics
        update_dynamics!(sim, integrator_step)
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
    u = outputs(sim.actuator)
    dynamics!(sim.dynamics, ẋ, x, u, t)
end

function update_dynamics!(sim, integrator)
    t_current = integrator.t
    ẋ = integrable_substate_derivative(sim.dynamics)
    xi = integrable_substate(sim.dynamics)
    x = state(sim.dynamics)
    x_vec = sim.dynamics.x_integrable_vector
    u = outputs(sim.actuator)

    state_modified = update!(sim.dynamics, ẋ, x, u, t_current)::Bool
    if state_modified
        copyto!(x_vec, xi)
        set_u!(integrator, x_vec)
    end

    # regardless of whether or not the state actually changed, set this flag to
    # recompute the dynamics at the beginning of the next integration step,
    # since the controls likely changed
    u_modified!(integrator, true)
end

function update_sensor_controller_actuator!(sim::Simulation)
    t_current = get(sim.simtime)
    update!(sim.sensor, outputs(sim.sensor), state(sim.sensor), state(sim.dynamics), t_current)
    update!(sim.controller, outputs(sim.controller), state(sim.controller), outputs(sim.sensor), t_current)
    update!(sim.actuator, outputs(sim.actuator), state(sim.actuator), outputs(sim.controller), t_current)
end

process_time_span(tspan::Tuple) = tspan[1], tspan[2]
process_time_span(tfinal::Real) = zero(typeof(tfinal)), tfinal
