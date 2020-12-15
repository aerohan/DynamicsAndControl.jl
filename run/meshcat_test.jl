using MeshCat
using GeometryBasics
using CoordinateTransformations
using Rotations

function visualize!(vis, data)
    state = data.truth.state
    t = state.time

    delete!(vis)
    DIR = "/Users/rohan/develop/explore/research/TrajOptPlots.jl/src"
    meshfile = joinpath(DIR,"..","data","meshes","cirrus","cirrus_scaled.obj")
    obj = MeshFileGeometry(meshfile)
    setobject!(vis[:aircraft], obj)
    settransform!(vis[:aircraft], compose(Translation(0,0,0.07),LinearMap( Quat(1.0, 0.0, 0.0, 0.0))))

    N = length(t)
    tf = last(t)
    fps = Int(floor(N/tf))
    @show fps
    anim = MeshCat.Animation(fps)

    q_world2ned = RotX(π)
    q_body2model = inv(RotZ(pi/2) * RotX(pi/2))

    for (idx, (t, r, q)) in enumerate(zip(state.time, state.r, state.q))
        #r_world = q_world2ned*(r + @SVector [0.0, 6.0, 0.0])
        r_world = q_world2ned*r
        q_ned2body = Quat(q...)
        #q_world2body = inv(q_world2ned*Quat(1.0, 0.0, 0.0, 0.0))
        #q_world2body = Quat(1.0, 0.0, 0.0, 0.0)
        atframe(anim, idx) do
            settransform!(vis[:aircraft],
                compose(Translation(r_world...), LinearMap(q_world2ned*q_ned2body*q_body2model))
                #compose(Translation(r_world...), LinearMap(RotX(π/2)*RotY(π/2)))
            )
        end
    end

    setanimation!(vis, anim)
end
