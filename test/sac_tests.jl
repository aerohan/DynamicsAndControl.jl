using Test
using BenchmarkTools

using DynamicsAndControl
import DynamicsAndControl
using StaticArrays

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

@dynamics ScalarTestDynamics{T} begin
    @integrable begin
        x::T
    end
end

@sensor ScalarTestSensor{T} begin
    @outputs begin
        x_sns::T
    end
end

@actuator ScalarTestActuator{T} begin
    @outputs begin
        u::T
    end
end

@controller ScalarTestController{T, I<:Int} begin
    @state begin
        counter::I
    end

    @outputs begin
        u::T
    end
end
