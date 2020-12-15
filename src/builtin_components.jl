@sensor NoSensor begin end
@controller NoController begin end
@actuator NoActuator begin end

initialize(::Type{NoSensor}, config) = (), (), NamedTuple()
initialize(::Type{NoController}, config) = (), (), NamedTuple()
initialize(::Type{NoActuator}, config) = (), (), NamedTuple()

# Pass the entire state as the sensor output
struct FullStateSensor{DS<:DynamicState} <: Sensor
    state::DS
end
initialize(::Type{FullStateSensor}, config) = (), (), NamedTuple()
FullStateSensor(_, _, _, _, dynamics) = FullStateSensor(state(dynamics))
outputs(f::FullStateSensor) = f.state
state(f::FullStateSensor) = nothing

# Pass all of the controller outputs directly to the dynamics
struct PassthroughActuator{CO<:SensorActuatorControllerOutputs} <: Actuator
    controller_outputs::CO
end
initialize(::Type{PassthroughActuator}, config) = (), (), NamedTuple()
PassthroughActuator(_, _, _, _, controller) = PassthroughActuator(outputs(controller))
outputs(p::PassthroughActuator) = p.controller_outputs
state(p::PassthroughActuator) = nothing
