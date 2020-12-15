using DynamicsAndControl
using StaticArrays
using Rotations
using OrdinaryDiffEq
import RobotZoo
import RobotDynamics

@dynamics YakAircraft{T} begin
    @integrable begin
        r::SVector{3, T} 
        v::SVector{3, T}
        q::SVector{4, T}
        ω::SVector{3, T}
    end
end

function DynamicsAndControl.initialize(::Type{YakAircraft}, config)
    yak = RobotZoo.YakPlane(Quat{Float64})

    r0 = @SVector [0.0, 0.0, -5.0]
    v0 = @SVector [5.0, 0.0, 0.0]
    q0 = @SVector [1.0, 0.0, 0.0, 0.0]
    ω0 = @SVector [0.0, 0.0, 0.0]

    return (r0, v0, q0, ω0), (), (yak=yak,)
end

function DynamicsAndControl.dynamics!(this::YakAircraft, ẋ, x, u, t)
    @unpack yak = static(this)

    x_static = @SVector [x.r[1], x.r[2], x.r[3], x.q[1], x.q[2], x.q[3], x.q[4],
                         x.v[1], x.v[2], x.v[3], x.ω[1], x.ω[2], x.ω[3]]

    u_static = @SVector [u.thr, u.ail, u.elev, u.rud]

    xd_static = RobotDynamics.dynamics(yak, x_static, u_static, t, this)

    ẋ.r = SVector{3}(@view xd_static[1:3])
    ẋ.q = SVector{4}(@view xd_static[4:7])
    ẋ.v = SVector{3}(@view xd_static[8:10])
    ẋ.ω = SVector{3}(@view xd_static[11:13])

    log!(this, :state, t, (r=x.r, v=x.v, q=x.q, ω=x.ω))
end

@controller YakController{T} begin
    @outputs begin
        thr::T
        ail::T
        elev::T
        rud::T
    end
end

DynamicsAndControl.initialize(::Type{YakController}, config) = (), (0.0, 0.0, 0.0, 0.0), NamedTuple()

function DynamicsAndControl.update!(this::YakController, u, _, x, t)
    trim = @SVector [41.6666, 106, 74.6519, 106]

    u.thr = trim[1]
    u.ail = trim[2]
    u.elev = trim[3] + 60
    u.rud = trim[4]

    if t > 5.0 && t < 5.2
        u.elev = trim[3] + 30
    end

    log!(this, (), t, (thr=u.thr, ail=u.ail, elev=u.elev, rud=u.rud))
end

