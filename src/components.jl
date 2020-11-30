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
    @capture(typename, T_{P__})
    typename_bare = T
    type_params = P

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
            if T == integrable_state_type
                integrable_state_type = :($integrable_state_type{$(intersected_type_params...)})
            elseif T == direct_state_type
                direct_state_type = :($direct_state_type{$(intersected_type_params...)})
            end
        end

        subblock
    end for subblock in blocks.args]...)

    dynamic_state_type = :($(Symbol(typename_bare, "DynamicState")){$(type_params...)})
    push!(type_params, :(NT <: NamedTuple))

    quote
        $blocks

        struct $(dynamic_state_type) <: DynamicState
            x_integrable::$(integrable_state_type)
            x_direct::$(direct_state_type)
        end

        struct $(typename_bare){$(type_params...)} <: Dynamics
            x::$(dynamic_state_type)
            static::NT
            telem::TelemetrySink
        end

        # constructor for initializing dynamics component from initial state named tuples
        function $(typename_bare)(x_integrable_namedtuple, x_direct_namedtuple, static, telem)
            istate = $(namify(integrable_state_type))(x_integrable_namedtuple...)
            dstate = $(namify(direct_state_type))(x_direct_namedtuple...)
            dynamic_state = $(namify(dynamic_state_type))(istate, dstate)

            istate_fields = fieldnames(typeof(istate))
            dstate_fields = fieldnames(typeof(dstate))
            @assert length(intersect(istate_fields, dstate_fields)) == 0 "field names of integrable and direct substates must not have duplicates"

            return $(typename_bare)(dynamic_state, static, telem)
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
