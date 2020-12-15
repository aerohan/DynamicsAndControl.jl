using Test
using BenchmarkTools

using DynamicsAndControl
import DynamicsAndControl
using StaticArrays
using OrdinaryDiffEq

# make sure all of these generate without errors
@sensor SensorFoo{T, N1, N2, S} begin
    @state begin
        a::T
        b::SVector{N1, T}
        c::T
        d::T
        e::MVector{N2, T}
    end

    @outputs begin
        f::S
        g::T
    end
end

@actuator ActuatorFoo{T, N1, N2, S} begin
    @state begin
        a::T
        b::SVector{N1, T}
        c::T
        d::T
        e::MVector{N2, T}
    end

    @outputs begin
        f::S
        g::T
    end
end

@controller ControllerFoo{T, N1, N2, S} begin
    @state begin
        a::T
        b::SVector{N1, T}
        c::T
        d::T
        e::MVector{N2, T}
    end

    @outputs begin
        f::S
        g::T
    end
end

@controller DynamicsFooNext{T, N1, N2, S} begin
    @state begin
        a::T
        b::SVector{N1, T}
        c::T
        d::T
        e::MVector{N2, T}
    end

    @outputs begin
        f::S
        g::T
    end
end

# test components together
@dynamics ScalarTestDynamics{T} begin
    @integrable begin
        val::T
    end
end
DynamicsAndControl.initialize(::Type{ScalarTestDynamics}, config) = (1.0,), (), NamedTuple()
function DynamicsAndControl.dynamics!(this::ScalarTestDynamics, ẋ, x, u, t)
    ẋ.val = -.05*x.val + u.ctrl
    #println("\tdynamics: t=$t, ẋ=$(ẋ.val), x=$(x.val), u=$(u.ctrl)")
end
function DynamicsAndControl.update!(this::ScalarTestDynamics, ẋ, x, u, t)
    #println("\tdynamics update, t=$t")
    log!(this, :state, t, (x=x.val, ẋ=ẋ.val, u=u.ctrl))
    return true
end

@sensor ScalarTestSensor{T} begin
    @outputs begin
        val_sns::T
    end
end
DynamicsAndControl.initialize(::Type{ScalarTestSensor}, config) = (), (1.0,), NamedTuple()
function DynamicsAndControl.update!(this::ScalarTestSensor, y, _, x, t)
    y.val_sns = x.val*1.2
    #println("\tsensor: t=$t, y=$(y.val_sns), x=$(x.val)")
end

@actuator ScalarTestActuator{T} begin
    @outputs begin
        ctrl::T
    end
end
DynamicsAndControl.initialize(::Type{ScalarTestActuator}, config) = (), (1.0,), NamedTuple()
function DynamicsAndControl.update!(this::ScalarTestActuator, u_out, _, u_in, t)
    u_out.ctrl = u_in.ctrl
    #println("\tactuator: t=$t, u=$(u_out.ctrl)")
end

@controller ScalarTestController{T, I<:Int} begin
    @state begin
        counter::I
    end

    @outputs begin
        ctrl::T
    end
end
DynamicsAndControl.initialize(::Type{ScalarTestController}, config) = (0,), (1.0,), NamedTuple()
function DynamicsAndControl.update!(this::ScalarTestController, u, control_state, y, t)
    u.ctrl = Float64(control_state.counter) + y.val_sns
    control_state.counter += 1
    #println("\tcontroller: t=$t, u=$(u.ctrl)")
end

function test_components()
    tstart, tfinal = 0.0, 10.0
    dt = 1.0
    simtime = DynamicsAndControl.SimTime(tstart)
    log_sink = DynamicsAndControl.LogDataSink()

    dynamics = DynamicsAndControl._setup(Dynamics, (:truth, ScalarTestDynamics, ()), simtime, dt, log_sink)
    sensor = DynamicsAndControl._setup(Sensor, (:sensor, ScalarTestSensor, ()), simtime, dt, dt, log_sink, dynamics)
    controller = DynamicsAndControl._setup(Controller, (:actuator, ScalarTestController, ()), simtime, dt, dt, log_sink, sensor)
    actuator = DynamicsAndControl._setup(Actuator, (:actuator, ScalarTestActuator, ()), simtime, dt, dt, log_sink, controller)
end

function test_sim()
    sim = Simulation( 
                         ( :truth, ScalarTestDynamics, () ), 
                         ( :sensor, ScalarTestSensor, () ), 
                         ( :controller, ScalarTestController, () ), 
                         ( :actuator, ScalarTestActuator, () ), 
                         10.0, RK4(), dt=1.0
                    )
    data = simulate(sim)
end

@dynamics ScalarTestDynamics2{T} begin
    @integrable begin
        val::T
    end
end
DynamicsAndControl.initialize(::Type{ScalarTestDynamics2}, config) = (1.0,), (), NamedTuple()
function DynamicsAndControl.dynamics!(this::ScalarTestDynamics2, ẋ, x, u, t)
    ẋ.val = -.05*x.val + u.ctrl
    #println("\tdynamics: t=$t, ẋ=$(ẋ.val), x=$(x.val), u=$(u.ctrl)")
end
function DynamicsAndControl.update!(this::ScalarTestDynamics2, ẋ, x, u, t)
    #println("\tdynamics update, t=$t")
    log!(this, :state, t, (x=x.val, ẋ=ẋ.val, u=u.ctrl))
    return true
end

@controller ScalarTestController2{T, I<:Int} begin
    @state begin
        counter::I
    end

    @outputs begin
        ctrl::T
    end
end
DynamicsAndControl.initialize(::Type{ScalarTestController2}, config) = (0,), (1.0,), NamedTuple()
function DynamicsAndControl.update!(this::ScalarTestController2, u, control_state, y, t)
    u.ctrl = Float64(control_state.counter) + y.val
    control_state.counter += 1
    log!(this, (), t, (ctrl=u.ctrl, counter=control_state.counter))
end

function test_passthrough_sensor_actuator()
    sim = Simulation( 
                         ( :truth, ScalarTestDynamics2, () ), 
                         ( :controller, ScalarTestController2, () ), 
                         10.0, RK4(), dt=1.0
                    )
    data = simulate(sim)
end

function test_control_dt()
    sim = Simulation( 
                         ( :truth, ScalarTestDynamics2, () ), 
                         ( :controller, ScalarTestController2, () ), 
                         100.0, RK4(), dt=1.0
                    )
    data1 = simulate(sim)

    sim = Simulation( 
                         ( :truth, ScalarTestDynamics2, () ), 
                         ( :controller, ScalarTestController2, () ), 
                         100.0, RK4(), dt=1.0, control_dt=5.0
                    )
    data2 = simulate(sim)

    @test all(diff(series(data1.controller.time)) .≈ 1.0)
    @test all(diff(series(data2.controller.time)) .≈ 5.0)
end

test_components()
test_sim()
test_passthrough_sensor_actuator()
test_control_dt()
