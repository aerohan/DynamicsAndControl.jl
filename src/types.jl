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

# Time
mutable struct SimTime{T}
    t::T
    t_dir::T
end
SimTime(t) = SimTime(t, one(typeof(t)))
set!(time::SimTime, t) = (time.t = t)
get(time::SimTime) = time.t
set_dir!(time::SimTime, dir) = (time.t_dir = dir)
dir(time::SimTime) = time.t_dir
