abstract type Component end

abstract type Dynamics <: Component end

abstract type IntegrableState end
abstract type DirectState end

abstract type SensorActuatorController <: Component end
abstract type SensorActuator <: SensorActuatorController end
abstract type Sensor <: SensorActuator end
abstract type Actuator <: SensorActuator end
abstract type Controller <: SensorActuatorController end

abstract type SensorActuatorControllerState end
abstract type SensorActuatorControllerOutputs end
