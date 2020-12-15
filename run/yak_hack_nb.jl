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

using Revise
using DynamicsAndControl
using OrdinaryDiffEq

using LinearAlgebra

using Plots

# +
include("yak_hack.jl")

sim = Simulation(
                    ( :truth, YakAircraft, () ),
                    ( :controller, YakController, () ),
                    10.0, RK4(), dt=0.02
               )

data, sol = simulate(sim)
# -

plot(
    plot(data.truth.state.time, data.truth.state.r, label=["x" "y" "z"], ylabel="NED position [m]"),
    plot(data.truth.state.time, data.truth.state.v, label=["x" "y" "z"],ylabel="NED velocity [m/s]"),
    plot(data.truth.debug.time, data.truth.debug.a_rin.*180/π, legend=false, ylabel="α [deg]"),
    plot(data.truth.state.time, data.truth.state.ω, label=["x" "y" "z"],ylabel="angular velocity [rad/s]"),
    plot(data.truth.state.time, data.truth.state.q, legend=false,ylabel="quaternion"),
    plot(data.truth.debug.time, data.truth.debug.F_aero, label=["x" "y" "z"],ylabel="body aero force [N]"),
    size=(1000, 600), xlabel="time [s]", fmt=:png
)

plot(data.controller.time, data.controller.thr, xlim=(0.0, 4.0))

data
