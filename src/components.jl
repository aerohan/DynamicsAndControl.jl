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

    # record the integration state type name
    get_istate_type(block) = begin
         @capture(block, mutable struct I_ <: IntegrableState fields__ end) || return nothing
         return I
    end
    integrable_state_types = [get_istate_type(block) for block in blocks.args]
    integrable_state_types = integrable_state_types[integrable_state_types .!= nothing]
    length(integrable_state_types) >= 1 || error("unable to parse integrable state definition, are there multiple @integrable sections?")
    integrable_state_type = length(integrable_state_types) == 1 ? integrable_state_types[1] : nothing

    # record the direct state type name
    get_dstate_type(block) = begin
         @capture(block, mutable struct I_ <: DirectState fields__ end) || return nothing
         return I
    end
    direct_state_types = [get_dstate_type(block) for block in blocks.args]
    direct_state_types = direct_state_types[direct_state_types .!= nothing]
    length(direct_state_types) >= 1 || error("unable to parse direct state definition, are there multiple @direct sections?")
    direct_state_type = length(direct_state_types) == 1 ? direct_state_types[1] : nothing

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
