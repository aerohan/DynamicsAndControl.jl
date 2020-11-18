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
