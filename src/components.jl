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

# Dynamics

macro state(typename, blocks)
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
            integrable_state_type = gensym("IntegrableState")
            return :($integrable_state_type <: IntegrableState)
        else
            return expr
        end
    end

    # set up the direct substate
    direct_state_type = nothing
    blocks = MacroTools.prewalk(blocks) do expr
        if @capture(expr, Placeholder <: DirectState)
            @assert direct_state_type == nothing "unable to parse, are there multiple @direct sections?"
            direct_state_type = gensym("DirectState")
            return :($direct_state_type <: DirectState)
        else
            return expr
        end
    end

    # propagate parametric types
    blocks = MacroTools.prewalk(x->@capture(x, T1_ <: S_) ? :($(T1){$(type_params...)} <: $(S)) : x, blocks)

    dynamic_state_type = gensym("DynamicState")
    push!(type_params, :(NT <: NamedTuple))
    quote
        $blocks

        struct $(dynamic_state_type) <: DynamicState
            x_integrable::$(integrable_state_type)
            x_direct::$(direct_state_type)
        end

        mutable struct $(typename_bare){$(type_params...)} <: Dynamics
            #x::$(dynamic_state_type)
            static::NT
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
