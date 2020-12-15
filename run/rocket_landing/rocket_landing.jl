# -*- coding: utf-8 -*-
# ---
# jupyter:
#   jupytext:
#     cell_metadata_filter: -all
#     formats: ipynb,jl:light
#     text_representation:
#       extension: .jl
#       format_name: light
#       format_version: '1.5'
#       jupytext_version: 1.4.2
#   kernelspec:
#     display_name: Julia 1.5.3
#     language: julia
#     name: julia-1.5
# ---

using DynamicsAndControl
using StaticArrays
using Rotations
using OrdinaryDiffEq

@dynamics PlanarRocketLanding{T} begin
    @integrable begin
        r::SVector{2, T} 
        v::SVector{2, T}
        θ::T
        ω::T
        m::T
    end

    @direct begin
        landed::Bool
    end
end

@controller RocketLandingController{T} begin
    @outputs begin
        δ_gim::T
        T_thrust::T
    end
end

function DynamicsAndControl.initialize(::Type{PlanarRocketLanding}, config)
    x0 = config.x0

    return (x0.r, x0.v, x0.θ, x0.ω, x0.m), (false,), config.params
end

function DynamicsAndControl.dynamics!(this::PlanarRocketLanding, ẋ, x, u, t)
    @unpack Isp, J = static(this)
    @unpack r, v, θ, ω, m = x
    @unpack δ_gim, T_thrust = u

    f_thrust = T_thrust*RotMatrix(θ)*@SVector([1.0, 0.0])
    f_grav = @SVector([0.0, -m*9.81])
    f_total = f_thrust + f_grav

    ẋ.r = v
    ẋ.v = f_total/m
    ẋ.θ = ω
    ẋ.ω = T_thrust*sin(δ_gim)/J
    ẋ.m = -T_thrust*Isp

    log!(this, :state, t, (a=ẋ.v, ω̇=ẋ.ω, r, v, θ, ω, m))
    log!(this, :forces, t, (;f_thrust, f_grav, f_total))
end

DynamicsAndControl.initialize(::Type{RocketLandingController}, config) = (), (0.0, 0.0), config

function DynamicsAndControl.update!(this::RocketLandingController, u, _, x, t)
    @unpack m = x
    @unpack T_W = static(this)

    T_thrust = m*9.81 * T_W
    δ_gim = 0.0

    @pack! u = T_thrust, δ_gim

    log!(this, :outputs, t, (;T_thrust, δ_gim))
end

dynamics_conf = ( x0 = (
                         r = @SVector([.5e3, 2e3]), 
                         v = @SVector([-50.0, -250.0]),
                         θ = -20*π/180,
                         ω = .3*π/180,
                         m = 1e4,
                    ),
                  params = (
                        Isp = 300*9.81,
                        J = 1e6,
                    )
                 )

control_conf = ( T_W = 1.1, )


sim = Simulation(
                    ( :truth, PlanarRocketLanding, dynamics_conf ),
                    ( :controller, RocketLandingController, control_conf ),
                    10.0, RK4(), dt=0.02
               )

data, _ = simulate(sim)

let f = data.truth.forces
    plot(f.time, f.f_total[1], xlabel="time", ylabel="f horz")
    plot!(f.time, f.f_total[2], xlabel="time", ylabel="f vert")
end
