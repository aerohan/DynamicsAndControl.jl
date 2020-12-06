function initialize(::Type{Component}, config) end

simtime(c::Component) = c.component_common.simtime
sim_dt(c::Component) = c.component_common.simulation_dt
component_dt(c::Component) = c.component_common.component_dt
namespace(c::Component) = c.component_common.namespace
log_sink(c::Component) = c.component_common.log_sink
static(c::Component) = c.component_common.static

function dynamics!(::Dynamics, ẋ, x, u, t) return nothing end
function update!(::Dynamics, ẋ, x, u, t) return false end

integrable_substate(dynamics::Dynamics) = getfield(dynamics.x, :x_integrable)
integrable_substate_derivative(dynamics::Dynamics) = getfield(dynamics.x, :ẋ_integrable)
direct_substate(dynamics::Dynamics) = getfield(dynamics.x, :x_direct)
state(dyn::Dynamics) = dyn.x

function update!(::SensorActuatorController, output, state, input, t) return nothing end
outputs(sac::SensorActuatorController) = sac.outputs
state(sac::SensorActuatorController) = sac.state

# Time
mutable struct SimTime{T}
    t::T
end
set!(time::SimTime, t) = (time.t = t)
get(time::SimTime) = time.t

struct ComponentCommon{T_t, NT}
    # Global sim time
    simtime::SimTime{T_t}
    simulation_dt::T_t
    component_dt::T_t

    # Auxiliary data and objects (static params and telem sink)
    static::NT
    log_sink::LogDataSink
    namespace::Symbol
end

# Dynamics

macro dynamics(typename, blocks)
    blocks, typename_bare, type_params = _component_definition_parse_expand(typename, blocks, __module__)

    # set up the integrable substate
    blocks, integrable_state_type = _configure_subcomponent(blocks, typename_bare, type_params, :IntegrableState, :IntegrableState, "@integrable")

    # set up the direct substate
    blocks, direct_state_type =     _configure_subcomponent(blocks, typename_bare, type_params, :DirectState, :DirectState, "@direct")

    # propagate parametric types
    if length(type_params) > 0
        dynamic_state_type = :($(Symbol(typename_bare, "DynamicState")){$(type_params...)})
    else
        dynamic_state_type = :($(Symbol(typename_bare, "DynamicState")))
    end

    push!(type_params, :(V<:AbstractVector))
    push!(type_params, :(T_xi0<:Tuple))
    push!(type_params, :(T_xd0<:Tuple))
    push!(type_params, :(NT<:NamedTuple))
    push!(type_params, :(T_t<:Real))

    quote
        $blocks

        struct $(dynamic_state_type) <: DynamicState
            x_integrable::$(_strip_super_typename(integrable_state_type))
            ẋ_integrable::$(_strip_super_typename(integrable_state_type))
            x_direct::$(_strip_super_typename(direct_state_type))
        end

        struct $(typename_bare){$(type_params...)} <: Dynamics
            # Dynamic state (superset of integrable and direct substates)
            x::$(dynamic_state_type)

            # Integrable state vector representation for ODE interface
            x_integrable_vector::V

            # Tuples holding initial state data
            x_initial_integrable::T_xi0
            x_initial_direct::T_xd0

            component_common::ComponentCommon{T_t, NT}
        end

        # constructor for initializing dynamics component from initial state tuples
        function $(typename_bare)(x_integrable_tuple_in, x_direct_tuple_in, component_common)
            return create_dynamics($(typename_bare), $(namify(integrable_state_type)), $(namify(direct_state_type)), $(namify(dynamic_state_type)),
                                   x_integrable_tuple_in, x_direct_tuple_in, component_common)
        end
    end |> esc
end

macro integrable(block)
    return quote
        mutable struct Placeholder <: IntegrableState
            $(block.args...)
        end
    end |> esc
end

macro direct(block)
    return quote
        mutable struct Placeholder <: DirectState
            $(block.args...)
        end
    end |> esc
end

# Sensor, Actuator, Controller
macro sensor(typename, block)
    return _sensor_actuator_controller_setup(typename, block, :Sensor, __module__)
end

macro actuator(typename, block)
    return _sensor_actuator_controller_setup(typename, block, :Actuator, __module__)
end

macro controller(typename, block)
    return _sensor_actuator_controller_setup(typename, block, :Controller, __module__)
end

macro state(block)
    return quote
        mutable struct Placeholder <: SensorActuatorControllerState
            $(block.args...)
        end
    end |> esc
end

macro outputs(block)
    return quote
        mutable struct Placeholder <: SensorActuatorControllerOutputs
            $(block.args...)
        end
    end |> esc
end

function _sensor_actuator_controller_setup(typename, blocks, component_supertype_name, macro_module)
    blocks, typename_bare, type_params = _component_definition_parse_expand(typename, blocks, macro_module)

    # set up the integrable substate
    blocks, state_type = _configure_subcomponent(blocks, typename_bare, type_params, :SensorActuatorControllerState, :State, "@state")

    # set up the direct substate
    blocks, output_type =     _configure_subcomponent(blocks, typename_bare, type_params, :SensorActuatorControllerOutputs, :Outputs, "@output")

    push!(type_params, :(T_x0<:Tuple))
    push!(type_params, :(NT<:NamedTuple))
    push!(type_params, :(T_t<:Real))

    quote
        $blocks

        struct $(typename_bare){$(type_params...)} <: $component_supertype_name
            # Main state/output data structure
            state::$(_strip_super_typename(state_type))
            outputs::$(_strip_super_typename(output_type))

            # Tuple holding initial state data
            state_initial::T_x0

            component_common::ComponentCommon{T_t, NT}
        end

        # constructor for initializing component from initial state/output tuples
        function $(typename_bare)(state_tuple_in, output_tuple_in, component_common)
            return create_sensor_actuator_controller($(typename_bare), $(namify(state_type)), $(namify(output_type)),
                                   state_tuple_in, output_tuple_in, component_common)
        end

        # constructor with argument for input component, ignored in this case
        function $(typename_bare)(state_tuple_in, output_tuple_in, component_common, input_component)
            return $(typename_bare)(state_tuple_in, output_tuple_in, component_common)
        end
    end |> esc
end

# does some initial parsing and macro expansion of the compoment definition macro expression
function _component_definition_parse_expand(typename, blocks, macro_module)
    # capture parametric types to propagate to subcomponents
    typename_bare = namify(typename)
    if @capture(typename, T_{P__})
        type_params = P
    else
        type_params = []
    end

    # expand subcomponent macros
    blocks = macroexpand(macro_module, blocks)

    return blocks, typename_bare, type_params
end

# configures the subcomponent generated code, by updating the struct name and handling the unspecified case
#   example subcomponent supertypes: IntegrableState, DirectState, SensorActuatorControllerOutputs, SensorActuatorControllerState
function _configure_subcomponent(blocks, component_typename_bare::Symbol, component_type_params, subcomponent_supertype::Symbol, subcomponent_suffix::Symbol, subcomponent_macro_name::AbstractString)
    subcomponent_type_name = nothing
    blocks = MacroTools.prewalk(blocks) do expr
        if @capture(expr, Placeholder <: Tsuper_) && Tsuper == subcomponent_supertype
            @assert subcomponent_type_name == nothing "unable to parse, are there multiple $subcomponent_macro_name sections?"
            subcomponent_type_name = Symbol(component_typename_bare, subcomponent_suffix)
            return :($subcomponent_type_name <: $subcomponent_supertype)
        else
            return expr
        end
    end

    # handle the case where the subcomponent macro is missing, this means we should just make an empty struct for the subcomponent
    if subcomponent_type_name == nothing
        subcomponent_type_name = Symbol(component_typename_bare, subcomponent_suffix)
        push!(blocks.args, :(mutable struct $subcomponent_type_name <: $subcomponent_supertype end))
    end

    # now update the subcomponent type name by propagating the correct type parameters to it
    blocks, subcomponent_type_name = _propagate_type_parameters(blocks, subcomponent_type_name, component_type_params)

    return blocks, subcomponent_type_name
end

# propagate the type parameters of the component, to the type name of the subcomponent 
#   for example: FooIntegrableState -> FooIntegrableState{T, N} 
# we need to intersect the type params of the component with the params
# actually used in this subcomponent so that we can call the subcomponent
# constructor and have julia infer the types
function _propagate_type_parameters(blocks, subcomponent_type_name, component_type_params)
    blocks = Expr(blocks.head, [begin 
        if @capture(subblock, mutable struct T_ <: Tsuper_ fields__ end) && T == subcomponent_type_name
            # the type parameters for this subcomponent should only include the
            # types actually used in the subcomponent's field types, so intersect
            # the parameters with the parameters in the field types
            field_types = [begin
                @capture(field, f_::Tfield_) || error("all fields must have concrete types")
                Tfield
            end for field in fields]

            intersected_type_params = [param for param in component_type_params if any(inexpr.(field_types, _strip_type_param(param)))]
            subblock = MacroTools.postwalk(x->@capture(x, T1_ <: S_) ? :($(T1){$(intersected_type_params...)} <: $(S)) : x, subblock)

            # update the substate type name with the intersected type parameters
            if length(intersected_type_params) > 0
                subcomponent_type_name = :($subcomponent_type_name{$(intersected_type_params...)})
            end
        end

        subblock
    end for subblock in blocks.args]...)
    
    return blocks, subcomponent_type_name
end

function create_dynamics(::Type{T_dyn}, ::Type{T_int}, ::Type{T_dir}, ::Type{T_dynstate}, 
                         x_integrable_tuple_in, x_direct_tuple_in, component_common) where {T_dyn<:Dynamics, T_int<:IntegrableState, T_dir<:DirectState, T_dynstate<:DynamicState}
    # create the integrable, direct, dynamic state data structures
    istate = T_int(x_integrable_tuple_in...)
    istate_deriv = T_int(x_integrable_tuple_in...)
    dstate = T_dir(x_direct_tuple_in...)
    dynamic_state = T_dynstate(istate, istate_deriv, dstate)

    # initialize the ODE interface state vector with the correct size and type
    n_integrable = integrable_size(dynamic_state)
    T_scalar = integrable_scalar_type(dynamic_state)
    xi_vector = Vector{T_scalar}(undef, n_integrable)

    istate_fields = fieldnames(typeof(istate))
    dstate_fields = fieldnames(typeof(dstate))
    @assert length(intersect(istate_fields, dstate_fields)) == 0 "field names of integrable and direct substates must not have duplicates"

    return T_dyn(dynamic_state, xi_vector, Tuple(x_integrable_tuple_in), Tuple(x_direct_tuple_in), component_common)
end

function create_sensor_actuator_controller(::Type{T_sac}, ::Type{T_state}, ::Type{T_out},
                         state_tuple_in, output_tuple_in, component_common) where {T_sac<:SensorActuatorController, T_state<:SensorActuatorControllerState, T_out<:SensorActuatorControllerOutputs}
    # create the state and output data structures
    state = T_state(state_tuple_in...)
    outputs = T_out(output_tuple_in...)

    return T_sac(state, outputs, state_tuple_in, component_common)
end

_strip_type_param(param::Symbol) = param
_strip_type_param(param::Expr) = @capture(param, Tp_ <: Tpsuper_) && return Tp
_strip_super_typename(expr) = MacroTools.postwalk(x->@capture(x, T1_ <: S_) ? :($T1) : x, expr)

# provide getproperty/setproperty! interface to DynamicState to access substate fields
Base.getproperty(x::DynamicState, sym::Symbol) = Base.getproperty(x, Val(sym))
@generated function Base.getproperty(x::DynamicState, ::Val{sym}) where sym
    istate_types = fieldnames(fieldtype(x, :x_integrable))
    dstate_types = fieldnames(fieldtype(x, :x_direct))

    if sym in istate_types
        :(Base.getproperty(getfield(x, :x_integrable), sym))
    elseif sym in dstate_types
        :(Base.getproperty(getfield(x, :x_direct), sym))
    else
        error("\"$sym\" does not match any field in the DynamicState (neither integrable nor direct substates)")
    end
end

Base.setproperty!(x::DynamicState, sym::Symbol, value) = Base.setproperty!(x, Val(sym), value)
@generated function Base.setproperty!(x::DynamicState, ::Val{sym}, value) where sym
    istate_types = fieldnames(fieldtype(x, :x_integrable))
    dstate_types = fieldnames(fieldtype(x, :x_direct))

    if sym in istate_types
        :(Base.setproperty!(getfield(x, :x_integrable), sym, value))
    elseif sym in dstate_types
        :(Base.setproperty!(getfield(x, :x_direct), sym, value))
    else
        error("\"$sym\" does not match any field in the DynamicState (neither integrable nor direct substates)")
    end
end

integrable_size(dynamics::Dynamics) = integrable_size(dynamics.x)
integrable_size(dynstate::DynamicState) = integrable_size(getfield(dynstate, :x_integrable))

function integrable_size(istate::IntegrableState)
    ftypes = fieldtypes(typeof(istate))
    return length(ftypes) > 0 ? sum([integrable_size(ftype) for ftype in ftypes]) : 0
end

#function integrable_size(::Type{T<:IntegrableState}) where T
#    ftypes = fieldtypes(T)
#    return sum([integrable_size(ftype) for ftype in ftypes])
#end

integrable_size(::Type{T}) where T = 1
integrable_size(::Type{SVector{N, T}}) where {T, N} = N
integrable_size(::Type{MVector{N, T}}) where {T, N} = N
integrable_size(::Type{SizedVector{N, T}}) where {T, N} = N
integrable_size(::Type{T}) where {T<:Vector} = error("only statically sized vectors (SVector, MVector, SizedVector) are allowed in the integrable state")

integrable_scalar_type(dynamics::Dynamics) = integrable_scalar_type(dynamics.x)
integrable_scalar_type(dynstate::DynamicState) = integrable_scalar_type(getfield(dynstate, :x_integrable))

function integrable_scalar_type(substate::IntegrableState)
    ftypes = fieldtypes(typeof(substate))
    return promote_type([integrable_scalar_type(ftype) for ftype in ftypes]...)
end

integrable_scalar_type(::Type{T}) where T = eltype(T)

@generated function set_state!(x::DynamicState, x_integrable_in, x_direct_in)
    out = quote
        # get the state data structures
        x_integrable_data = getfield(x, :x_integrable)
        x_direct_data = getfield(x, :x_direct)

        # ensure the state input is a tuple (may be passed in as named tuple
        # for clarity, but only looking at positional arguments here)
        x_integrable_in = Tuple(x_integrable_in)
        x_direct_in = Tuple(x_direct_in)
    end

    T_xi = fieldtype(x, :x_integrable)
    for (index, field) in enumerate(fieldnames(T_xi))
        line = :(setfield!(x_integrable_data, $(QuoteNode(field)), x_integrable_in[$index]))
        push!(out.args, line)
    end

    T_xd = fieldtype(x, :x_direct)
    for (index, field) in enumerate(fieldnames(T_xd))
        line = :(setfield!(x_direct_data, $(QuoteNode(field)), x_direct_in[$index]))
        push!(out.args, line)
    end

    return out
end

@generated function Base.copyto!(xi_vector::AbstractVector, xi_dyn::IntegrableState)
    out = :()
    istart = 1
    for (name, type) in zip(fieldnames(xi_dyn), fieldtypes(xi_dyn))
        iend = istart + integrable_size(type) - 1
        line = :(view(xi_vector, $(istart:iend)) .= getfield(xi_dyn, $(QuoteNode(name))))
        push!(out.args, line)
        istart = iend + 1
    end
    push!(out.args, :(return nothing))

    return out
end

@generated function Base.copyto!(xi_dyn::IntegrableState, xi_vector::AbstractVector)
    out = :()
    istart = 1
    for (name, type) in zip(fieldnames(xi_dyn), fieldtypes(xi_dyn))
        iend = istart + integrable_size(type) - 1
        line = :(copyto!(xi_dyn, $(QuoteNode(name)), view(xi_vector, $(istart:iend)), $type))
        push!(out.args, line)
        istart = iend + 1
    end
    push!(out.args, :(return nothing))

    return out
end

# methods for copying a specific integrable state field from the ODE-interface
# vector to the integrable state data structure
#
# custom types can be placed inside the integrable state by satisfying the following interface:
#   1. define an "integrable_size" method
#   2. define a "integrable_scalar_type" method
#   3. define a "copyto!" method with the type signature matching below

# unless we have a specific method for deserializing, assume the view subarray
# is 1-element long and set the corresponding field in the state data struct to
# it 
function Base.copyto!(xi_dyn::IntegrableState, fieldname, xi_vec_view::SubArray, ::Type{T_field}) where {T_field}
    setfield!(xi_dyn, fieldname, T_field(xi_vec_view[1]))
    return nothing
end

# SVector (immutable) case
function Base.copyto!(xi_dyn::IntegrableState, fieldname, xi_vec_view::SubArray, ::Type{T_field}) where {T_field<:SVector}
    setfield!(xi_dyn, fieldname, T_field(xi_vec_view))
    return nothing
end

# MVector/SizedVector (mutable) case
function Base.copyto!(xi_dyn::IntegrableState, fieldname, xi_vec_view::SubArray, ::Type{T_field}) where {T_field<:Union{MVector, SizedVector}}
    getfield(xi_dyn, fieldname) .= xi_vec_view
    return nothing
end
