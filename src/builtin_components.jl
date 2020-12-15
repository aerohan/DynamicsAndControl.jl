@sensor NoSensor begin end
@controller NoController begin end
@actuator NoActuator begin end

initialize(::Type{NoSensor}, config) = (), (), NamedTuple()
initialize(::Type{NoController}, config) = (), (), NamedTuple()
initialize(::Type{NoActuator}, config) = (), (), NamedTuple()

# Pass the entire state as the sensor output
struct FullStateSensor{DS<:DynamicState,P<:PeriodicFixed,CD<:ComponentData} <: Sensor
    state::DS
    component_data::CD
    periodic::P
end
initialize(::Type{FullStateSensor}, config) = (), (), NamedTuple()
FullStateSensor(_, _, periodic, component_data, dynamics) = FullStateSensor(state(dynamics), 
                                                                            component_data, periodic)
outputs(f::FullStateSensor) = f.state
state(f::FullStateSensor) = nothing
periodic(f::FullStateSensor) = f.periodic
sim_dt(f::FullStateSensor) = f.component_data.simulation_dt

# Pass all of the controller outputs directly to the dynamics
struct PassthroughActuator{CO<:SensorActuatorControllerOutputs,P<:PeriodicFixed,CD<:ComponentData} <: Actuator
    controller_outputs::CO
    component_data::CD
    periodic::P
end
initialize(::Type{PassthroughActuator}, config) = (), (), NamedTuple()
PassthroughActuator(_, _, periodic, component_data, controller) = PassthroughActuator(outputs(controller), 
                                                                                      component_data, periodic)
outputs(p::PassthroughActuator) = p.controller_outputs
state(p::PassthroughActuator) = nothing
periodic(p::PassthroughActuator) = p.periodic
sim_dt(p::PassthroughActuator) = p.component_data.simulation_dt
