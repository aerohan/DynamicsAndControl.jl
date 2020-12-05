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

