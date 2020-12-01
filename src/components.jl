abstract type Component end

abstract type Dynamics <: Component end
abstract type Control <: Component end
abstract type Sensing <: Component end

abstract type DynamicState end
abstract type IntegrableState end
abstract type DirectState end

abstract type ControlState end
abstract type ControlOutputs end

abstract type SensorState end
abstract type SensorOutputs end

function initialize(::Type{Component}, config) end

# Dynamics

macro dynamics(typename, blocks)
    # capture parametric types to propagate to substates
    typename_bare = namify(typename)
    if @capture(typename, T_{P__})
        type_params = P
    else
        type_params = []
    end

    # expand substate macros
    blocks = macroexpand(__module__, blocks)

    # set up the integrable substate
    integrable_state_type = nothing
    blocks = MacroTools.prewalk(blocks) do expr
        if @capture(expr, Placeholder <: IntegrableState)
            @assert integrable_state_type == nothing "unable to parse, are there multiple @integrable sections?"
            integrable_state_type = Symbol(typename_bare, "IntegrableState")
            return :($integrable_state_type <: IntegrableState)
        else
            return expr
        end
    end

    # handle the unspecified case
    if integrable_state_type == nothing
        integrable_state_type = Symbol(typename_bare, "IntegrableState")
        push!(blocks.args, :(mutable struct $integrable_state_type <: IntegrableState end))
    end

    # set up the direct substate
    direct_state_type = nothing
    blocks = MacroTools.prewalk(blocks) do expr
        if @capture(expr, Placeholder <: DirectState)
            @assert direct_state_type == nothing "unable to parse, are there multiple @direct sections?"
            direct_state_type = Symbol(typename_bare, "DirectState")
            return :($direct_state_type <: DirectState)
        else
            return expr
        end
    end

    # handle the unspecified case
    if direct_state_type == nothing
        direct_state_type = Symbol(typename_bare, "DirectState")
        push!(blocks.args, :(mutable struct $direct_state_type <: DirectState end))
    end

    # propagate parametric types
    blocks = Expr(blocks.head, [begin 
        if @capture(subblock, mutable struct T_ <: Tsuper_ fields__ end)
            # the type parameters for this substate should only include the
            # types actually used in the substate's field types, so intersect
            # the parameters with the parameters in the field types
            field_types = [begin
                @capture(field, f_::Tfield_) || error("all fields must have concrete types")
                Tfield
            end for field in fields]
            intersected_type_params = [param for param in type_params if any(inexpr.(field_types, param))]
            subblock = MacroTools.postwalk(x->@capture(x, T1_ <: S_) ? :($(T1){$(intersected_type_params...)} <: $(S)) : x, subblock)

            # update the substate type name with the intersected type parameters
            if length(intersected_type_params) > 0
                if T == integrable_state_type
                    integrable_state_type = :($integrable_state_type{$(intersected_type_params...)})
                elseif T == direct_state_type
                    direct_state_type = :($direct_state_type{$(intersected_type_params...)})
                end
            end
        end

        subblock
    end for subblock in blocks.args]...)

    if length(type_params) > 0
        dynamic_state_type = :($(Symbol(typename_bare, "DynamicState")){$(type_params...)})
    else
        dynamic_state_type = :($(Symbol(typename_bare, "DynamicState")))
    end

    push!(type_params, :(V<:AbstractVector))
    push!(type_params, :(T_xi0<:Tuple))
    push!(type_params, :(T_xd0<:Tuple))
    push!(type_params, :(NT<:NamedTuple))

    quote
        $blocks

        struct $(dynamic_state_type) <: DynamicState
            x_integrable::$(integrable_state_type)
            x_direct::$(direct_state_type)
        end

        struct $(typename_bare){$(type_params...)} <: Dynamics
            # Dynamic state (superset of integrable and direct substates)
            x::$(dynamic_state_type)

            # Integrable state vector representation for ODE interface
            x_integrable_vector::V

            # Tuples holding initial state data
            x_initial_integrable::T_xi0
            x_initial_direct::T_xd0

            # Auxiliary data and objects (static params and telem sink)
            static::NT
            telem::TelemetrySink
        end

        # constructor for initializing dynamics component from initial state tuples
        function $(typename_bare)(x_integrable_tuple_in, x_direct_tuple_in, static, telem)
            return create_dynamics($(typename_bare), $(namify(integrable_state_type)), $(namify(direct_state_type)), $(namify(dynamic_state_type)),
                                   x_integrable_tuple_in, x_direct_tuple_in, static, telem)
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

macro state(block)
    return quote
        mutable struct Placeholder <: PlaceholderState
            $(block.args...)
        end
    end |> esc
end

macro outputs(block)
    return quote
        mutable struct Placeholder <: PlaceholderOutputs
            $(block.args...)
        end
    end |> esc
end

function create_dynamics(::Type{T_dyn}, ::Type{T_int}, ::Type{T_dir}, ::Type{T_dynstate}, 
                         x_integrable_tuple_in, x_direct_tuple_in, static, telem) where {T_dyn<:Dynamics, T_int<:IntegrableState, T_dir<:DirectState, T_dynstate<:DynamicState}
    # create the integrable, direct, dynamic state data structures
    istate = T_int(x_integrable_tuple_in...)
    dstate = T_dir(x_direct_tuple_in...)
    dynamic_state = T_dynstate(istate, dstate)

    # initialize the ODE interface state vector with the correct size and type
    n_integrable = integrable_size(dynamic_state)
    T_scalar = integrable_scalar_type(dynamic_state)
    xi_vector = Vector{T_scalar}(undef, n_integrable)

    istate_fields = fieldnames(typeof(istate))
    dstate_fields = fieldnames(typeof(dstate))
    @assert length(intersect(istate_fields, dstate_fields)) == 0 "field names of integrable and direct substates must not have duplicates"

    return T_dyn(dynamic_state, xi_vector, Tuple(x_integrable_tuple_in), Tuple(x_direct_tuple_in), static, telem)
end


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

integrable_substate(dynamics::Dynamics) = getfield(dynamics.x, :x_integrable)
direct_substate(dynamics::Dynamics) = getfield(dynamics.x, :x_direct)

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

function integrable_scalar_type(substate::Union{IntegrableState, DirectState})
    ftypes = fieldtypes(typeof(substate))
    return promote_type([integrable_scalar_type(ftype) for ftype in ftypes]...)
end

integrable_scalar_type(::Type{T}) where T = T
integrable_scalar_type(::Type{SVector{N, T}}) where {T, N} = T
integrable_scalar_type(::Type{MVector{N, T}}) where {T, N} = T
integrable_scalar_type(::Type{SizedVector{N, T}}) where {T, N} = T
integrable_scalar_type(::Type{T}) where {T<:Vector} = error("only statically sized vectors (SVector, MVector, SizedVector) are allowed in the integrable state")

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
