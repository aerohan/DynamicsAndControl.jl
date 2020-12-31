function differentiable_dynamics(::Type{D}, dynamics_config) where {D<:Dynamics}
    _, dstate_initial, static_dyn = initialize(D, dynamics_config)
    simtime = SimTime(NaN)
    component_data = ComponentData(simtime, NaN, NaN, static_dyn, SimulationLogger(LogDataSink(), simtime), Symbol())
    T_int = substate_types(D)[1]
    dynamics_func! = (ẋ, x, u, t) -> begin
        # this allocates a new dynamics object every call. not ideal, but not
        # trivial to fix this because we don't know the precise types of the
        # dual numbers beforehand. this also makes this function type unstable,
        # so this function is best for prototyping and not necessarily for
        # maximum performance
        dynamics = D(create_integrable_state_tuple(x, Val(T_int)), dstate_initial, component_data)
        copyto!(integrable_substate(dynamics), x)
        ẋ_dyn = integrable_substate_derivative(dynamics)
        x_dyn = state(dynamics)
        dynamics!(dynamics, ẋ_dyn, x_dyn, u, t)
        copyto!(ẋ, integrable_substate_derivative(dynamics))
    end

    return dynamics_func!
end

@generated function create_integrable_state_tuple(x::AbstractVector, ::Val{T_int}) where {T_int<:IntegrableState}
    out = Expr(:tuple)
    istart = 1
    for (name, type) in zip(fieldnames(T_int), fieldtypes(T_int))
        iend = istart + integrable_size(type) - 1
        # assumes value can be constructed like following example: T(x[1:3]...)
        item = :($(QuoteNode(type))(x[$(istart:iend)]...))
        push!(out.args, item)
        istart = iend + 1
    end
    return out
end
