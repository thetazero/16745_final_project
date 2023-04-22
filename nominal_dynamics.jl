using SatellitePlayground
"""
Returns the dynamics at a certain position, time
"""
function nominal_attitude_dynamics(x, state, J, time, control)
    ω, q = x[1:3], x[4:7]
    ᵇmagnetic = world_to_body(state, SatellitePlayground.IGRF13(state.position, time))
    u = cross(control, ᵇmagnetic)
    return [
        J \ (u - cross(ω, J * ω))
        qdot(q, ω)
    ]
end
