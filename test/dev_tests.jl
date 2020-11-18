using DynamicsAndControl: @state, @integrable, @direct, IntegrableState, DirectState, DynamicState, Dynamics
using StaticArrays

@state DynamicsType{T, N, S} begin
    @integrable begin
        x::Float64
        y::SVector{N, T}
        z::Float64
        w::T
    end

    @direct begin
        z::S
    end
end
