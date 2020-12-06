@sensor NoSensor begin end
@controller NoController begin end
@actuator NoActuator begin end

initialize(::Type{NoSensor}, config) = (), (), NamedTuple()
initialize(::Type{NoController}, config) = (), (), NamedTuple()
initialize(::Type{NoActuator}, config) = (), (), NamedTuple()
