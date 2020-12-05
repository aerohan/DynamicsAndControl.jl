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

function DynamicsAndControl.initialize(::Type{SensorFoo}, init_config)
    initial_state = (a=1.0, b=(@SVector [1.0, 2.0, 3.0]), c=2.0, w=init_config.w0, e=(@MVector [2.0, 5.1, 7.2, 1.4]))
    initial_output = (f=false, g=1.0)
    static = init_config

    # returns a tuple of ([initial integrable state], [initial direct state], [configuration parameters])
    return initial_state, initial_output, static
end

function test()

end
