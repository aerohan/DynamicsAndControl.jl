using BenchmarkTools

struct Simulation{D<:Dynamics}
    # Components
    dynamics::D
    #sensing::S where S<:Sensing
    #control::C where C<:Control

    # Auxiliary
    telem::TelemetrySink
end

function Simulation(dynamics_args)
                    #sensing_args,
                    #control_args)

    telem = TelemetrySink()

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
    dynamics = T_dyn(istate_initial, dstate_initial, static_dyn, telem)

    return Simulation(dynamics, telem)
end

function simulate(sim::Simulation, tspan)
    tstart, tfinal = process_time_span(tspan)

    set_state!(sim.dynamics.x, sim.dynamics.x_initial_integrable, sim.dynamics.x_initial_direct)

    return nothing
end

function dynamics_ode_interface(sim, ẋ, x, _, t)
    copyto!(sim.dynamics.x, x)
    compute_ẋ!(sim)
    copyto!(ẋ, sim.dynamics.ẋ)
end

process_time_span(tspan::Tuple) = tspan[1], tspan[2]
process_time_span(tfinal::Real) = 0, tfinal
