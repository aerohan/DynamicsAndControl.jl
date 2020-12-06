# telemetry is generated in "flows", which are sets of channels that are
# telemetered/logged together and at the same frequency, so they therefore have
# a consistent timebase. each flow is identified by a tuple of symbols, to
# facilitate organizing flows in a hierarchy and for managing complex datasets.
# the data sink accepts a set of channel values (at a particular time instance)
# and a flow id, and logs the data accordingly. the data dict contained within
# the data sink maps the flow id to a dict of channel name symbols (keys) and
# vector of channel values (values).
#
# for example, we may want to log channels :time, :altitude, :density,
# :pressure, :v_wind_ned to the (:truth, :atmosphere) flow. we then could, for
# example, plot the :altitude against :v_wind_ned from the (:truth,
# :atmosphere) flow.
struct LogDataFlowId{N}
    id::NTuple{N, Symbol}
end
LogDataFlowId(sym::Symbol) = LogDataFlowId((sym,))

mutable struct LogDataSink
    data::Dict{LogDataFlowId, Dict{Symbol, Vector}}
end
LogDataSink() = LogDataSink(Dict{LogDataFlowId, Dict{Symbol, Vector}}())

function log!(component::Component, flow, t, current_data::NamedTuple)
    make_tuple(flow::Symbol) = (flow,)
    make_tuple(flow) = flow

    augmented_flow_namespace = (namespace(component), make_tuple(flow)...)

    # log at either the end or start of an integration step
    if get(simtime(component)) + sim_dt(component) ≈ t || get(simtime(component)) ≈ t
        log!(log_sink(component), augmented_flow_namespace, t, current_data)
    end
end

function log!(log_sink::LogDataSink, flow, t, current_data::NamedTuple)
    flow = LogDataFlowId(flow)

    # at the first time, register this flow
    if !haskey(log_sink.data, flow)
        log_sink.data[flow] = Dict{Symbol, Vector}()
        for (k, T) in zip(keys(current_data), fieldtypes(typeof(current_data)))
            log_sink.data[flow][k] = T[]
            k != :time || error("\"time\" is a reserved channel name for logging")
        end
        log_sink.data[flow][:time] = typeof(t)[]
    end

    # log the data at this time
    log!(log_sink.data[flow], t, current_data)

    return nothing
end

function log!(flow_data::Dict{Symbol, Vector}, t, current_data)
    # pop all values that should be replaced by the data at the current time
    t_vec = flow_data[:time]::Vector{typeof(t)}
    while length(t_vec) > 0 && t_vec[end] >= t
        log_pop!(flow_data, t, current_data)
    end

    # add the latest data
    log_push!(flow_data, t, current_data)

    return nothing
end

@generated function log_push!(flow_data, t, current_data)
    out = :()
    push!(out.args, :(push!(flow_data[:time]::Vector{typeof(t)}, t)))
    for (name, type) in zip(fieldnames(current_data), fieldtypes(current_data))
        push!(out.args, :(push!(flow_data[$(QuoteNode(name))]::Vector{$type}, current_data.$name)))
    end
    return out
end

@generated function log_pop!(flow_data, t, current_data)
    out = :()
    push!(out.args, :(pop!(flow_data[:time]::Vector{typeof(t)})))
    for (name, type) in zip(fieldnames(current_data), fieldtypes(current_data))
        push!(out.args, :(pop!(flow_data[$(QuoteNode(name))]::Vector{$type})))
    end
    return out
end

struct SimulationDataset
    data::LogDataSink
end

struct SimulationDataSubset{N}
    dataset::SimulationDataset
    flow_sub_id::NTuple{N, Symbol}
end

struct FlowData
    data::Dict{Symbol, Vector}
end

function Base.getproperty(datasubset::SimulationDataSubset, name::Symbol)
    sub_id = getfield(datasubset, :flow_sub_id)
    dataset = getfield(datasubset, :dataset)
    full_id = (sub_id..., name)
    flow_id = LogDataFlowId(full_id)
    if flow_id in keys(getfield(dataset, :data).data)
        return FlowData(getfield(dataset, :data).data[flow_id])
    else
        return SimulationDataSubset(dataset, full_id)
    end
end
Base.getproperty(dataset::SimulationDataset, name::Symbol) = getproperty(SimulationDataSubset(dataset, ()), name)

Base.getproperty(flow_data::FlowData, name::Symbol) = getfield(flow_data, :data)[name]

