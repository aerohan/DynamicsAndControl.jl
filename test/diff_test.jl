using DynamicsAndControl
using StaticArrays
import ForwardDiff
using BenchmarkTools
using Test

@dynamics SimpleOscillator2D{T} begin
    @integrable begin
        r::SVector{2, T}
        v::SVector{2, T}
    end
end

DynamicsAndControl.initialize(::Type{SimpleOscillator2D}, config) = (@SVector(zeros(2)), @SVector(zeros(2))), (), config

function DynamicsAndControl.dynamics!(this::SimpleOscillator2D, ẋ, x, u, t)
    ẋ.r = x.v
    ẋ.v = - x.r / static(this).m
end

function dynamics_simple!(ẋ, x, p, t)
    ẋ[1:2] = x[3:4]
    ẋ[3:4] = -x[1:2] / p.m
end

function dynamics_static(x, p, t)
    r = SVector{2}(x[1], x[2])
    v = SVector{2}(x[3], x[4])

    @SVector [v[1], v[2], -r[1]/p.m, -r[2]/p.m]
end

function test()
    x_test = rand(4)
    ẋ_test = similar(x_test)

    params = (;m = 2.0)

    @btime ForwardDiff.jacobian(x->dynamics_static(x, $params, 0.0), $x_test)
    j1 = ForwardDiff.jacobian(x->dynamics_static(x, params, 0.0), x_test)

    j2 = similar(j1)
    @btime ForwardDiff.jacobian!($j2, (ẋ, x)->dynamics_simple!(ẋ, x, $params, 0.0), $ẋ_test, $x_test)
    ForwardDiff.jacobian!(j2, (ẋ, x)->dynamics_simple!(ẋ, x, params, 0.0), ẋ_test, x_test)

    j3 = similar(j1)
    dc_dynamics! = DynamicsAndControl.differentiable_dynamics(SimpleOscillator2D, params)
    @btime ForwardDiff.jacobian!($j3, (ẋ, x)->$dc_dynamics!(ẋ, x, (), 0.0), $ẋ_test, $x_test)
    ForwardDiff.jacobian!(j3, (ẋ, x)->dc_dynamics!(ẋ, x, (), 0.0), ẋ_test, x_test)

    display(j1)
    display(j2)
    display(j3)

    @test j1 ≈ j2
    @test j2 ≈ j3
end

test()
