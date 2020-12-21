###########################
# Simulation Data Logging #
###########################
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

function reset!(log_sink::LogDataSink)
    for (id, data_table) in log_sink.data
        for sym in collect(keys(data_table))
            data_v = data_table[sym]
            data_table[sym] = eltype(data_v)[]
        end
    end
end

struct SimulationLogger{T_t}
    log_sink::LogDataSink
    simtime::SimTime{T_t}
end

function _flow_full_namespace(component::Component, flow_spec)
    make_tuple(flow::Symbol) = (flow,)
    make_tuple(flow) = flow
    return (namespace(component), make_tuple(flow_spec)...)
end

function log!(component::Component, flow, t, current_data::NamedTuple)
    # log at either the end or start of an integration step
    if get(simtime(component)) + sim_dt(component) ≈ t || get(simtime(component)) ≈ t
        log!(logger(component), _flow_full_namespace(component, flow), t, current_data)
    end
end
log!(component::SensorActuatorController, flow, current_data::NamedTuple) = log!(logger(component), _flow_full_namespace(component, flow), current_data)

log!(logger::SimulationLogger, flow, t, current_data) = log!(logger.log_sink, flow, t, current_data, dir(logger.simtime))
log!(logger::SimulationLogger, flow, current_data) = log!(logger.log_sink, flow, get(logger.simtime), current_data, dir(logger.simtime))

function log!(log_sink::LogDataSink, flow, t, current_data::NamedTuple, t_dir=one(typeof(t)))
    flow = LogDataFlowId(flow)

    # if this is the first time around, register this flow
    if !haskey(log_sink.data, flow)
        log_sink.data[flow] = Dict{Symbol, Vector}()
        for (k, T) in zip(keys(current_data), fieldtypes(typeof(current_data)))
            k != :time || error("\"time\" is a reserved channel name for logging")
            log_sink.data[flow][k] = T[]
        end
        log_sink.data[flow][:time] = typeof(t)[]
    end

    # log the data at this time
    log!(log_sink.data[flow], t, t_dir, current_data)

    return nothing
end

function log!(flow_data::Dict{Symbol, Vector}, t, t_dir, current_data)
    # pop all values that should be replaced by the data at the current time,
    # supporting both forwards-in-time and backwards-in-time
    # integration/logging order
    t_vec = flow_data[:time]::Vector{typeof(t)}
    while length(t_vec) > 0 && ((t_dir >= zero(t) && t_vec[end] >= t) || (t_dir <= zero(t) && t_vec[end] <= t))
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

############################
# Simulation Data Handling #
############################
struct SimulationDataset
    data::LogDataSink
end

struct SimulationDataSubset{N}
    dataset::SimulationDataset
    flow_sub_id::NTuple{N, Symbol}
end
SimulationDataSubset(s::SimulationDataSubset, flow_sub_id) = SimulationDataSubset(dataset(s), flow_sub_id)
dataset(s::SimulationDataSubset) = getfield(s, :dataset)
data(s::SimulationDataSubset) = getfield(dataset(s), :data).data
id(s::SimulationDataSubset) = getfield(s, :flow_sub_id)

function Base.show(io::IO, datasubset::SimulationDataSubset{N}) where N
    println(io, "Simulation data set with immediate namespaces:")
    sim_data = data(datasubset)
    flow_id = LogDataFlowId(id(datasubset))
    print(io, "\t")
    println(io, unique([k.id[N+1] for k in keys(sim_data) if _startswith(k, flow_id)]))
end
Base.show(io::IO, dataset::SimulationDataset) = show(io, SimulationDataSubset(dataset, ()))

struct FlowData
    data::Dict{Symbol, Vector}
end
data(fd::FlowData) = getfield(fd, :data)

struct DataSeries{T}
    series::Vector{T}
end
Base.show(io::IO, m::MIME"text/plain", ds::DataSeries) = show(io, m, ds.series)

_startswith(id1::LogDataFlowId, id2::LogDataFlowId) = all([ns1 == ns2 for (ns1, ns2) in zip(id1.id, id2.id)])

function Base.getproperty(datasubset::SimulationDataSubset, name::Symbol)
    full_id = (id(datasubset)..., name)
    sim_data = data(datasubset)
    flow_id = LogDataFlowId(full_id)
    if flow_id in keys(sim_data)
        return FlowData(sim_data[flow_id])
    elseif any([_startswith(k, flow_id) for k in keys(sim_data)])
        return SimulationDataSubset(datasubset, full_id)
    else
        error("\"$full_id\" not found in simulation dataset")
    end
end
Base.getproperty(dataset::SimulationDataset, name::Symbol) = getproperty(SimulationDataSubset(dataset, ()), name)

Base.getproperty(flow_data::FlowData, name::Symbol) = DataSeries(getfield(flow_data, :data)[name])

function Base.show(io::IO, flow_data::FlowData)
    println(io, "Simulation data flow with channels:")
    print(io, "\t")
    println(io, keys(data(flow_data)))
end

@recipe _handle_dataseries(::Type{T}, val::T) where{T<:DataSeries} = _plotting_process(val.series)
_plotting_process(x) = x
_plotting_process(x::Vector{T}) where {T<:AbstractVector} = reduce(hcat, x)'

Base.getindex(ds::DataSeries{T}, i::Int) where {T<:AbstractVector} = map(val->val[i], ds.series)
Base.getindex(ds::DataSeries{T}) where {T<:Real} = ds.series

Base.broadcasted(f, ds::DataSeries, args...) = Base.broadcasted(f, ds.series, args...)
Base.iterate(ds::DataSeries, args...) = iterate(ds.series, args...)
Base.length(ds::DataSeries, args...) = length(ds.series, args...)
Base.first(ds::DataSeries, args...) = first(ds.series, args...)
Base.last(ds::DataSeries, args...) = last(ds.series, args...)

series(ds::DataSeries) = ds.series
