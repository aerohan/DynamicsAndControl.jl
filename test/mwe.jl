using DynamicsAndControl: @state, @integrable, @direct, IntegrableState, DirectState, DynamicState, Dynamics

@dynamics AircraftDynamics{T} begin
    @integrable begin
        r::Vec3{T}
        v::Vec3{T}
        q::Quat{T}
        ω::Vec3{T}
    end

    @direct begin
        landed::Bool
    end
end

@sensing AircraftGaussianStateSensor{T} begin
    @output begin
        r_est::Vec3{T}
        v_est::Vec3{T}
        q_est::Quat{T}
        ω_est::Vec3{T}
    end

    @state begin
        r_bias::Vec3{T}
        v_bias::Vec3{T}
        ω_bias::Vec3{T}
    end
end

@control SimpleLQRAircraftControl{T} begin
    @output begin
        δ_elevon::MVector{2, T}
        δ_thrust::MVector{2, T}
        T_thrust::MVector{2, T}
    end
end

function initialize(::Type{AircraftDynamics}, init_config)
    istate_0 = (r=init_config.r0, v=init_config.v0, q=init_config.q0, ω=init_config.ω0)
    dstate_0 = (landed=false,)

    # returns a tuple of ([initial integrable state], [initial direct state], [configuration parameters])
    return istate_0, dstate_0, init_config
end

function dynamics!(this::AircraftDynamics, ẋ, x, u, t)
    @unpack r, v, q, ω = x
    @unpack f = u
    @unpack mass = config(this)

    ẋ.r = v
    ẋ.v = f/mass

    telemeter(this, :main, t, @channels begin
        ẋ.r => ṙ
        ẋ.v => v̇
        α
    end)
end

function control!(this::AircraftDynamics, u, control_state, y)

end
