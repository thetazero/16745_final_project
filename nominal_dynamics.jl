using SatellitePlayground
"""
Returns the dynamics at a certain position, time
"""
function nominal_attitude_dynamics(state, time, control)
    ᵇmagnetic = world_to_body(state, SatellitePlayground.IGRF13(state.position, time))
    u = cross(control, ᵇmagnetic)
    return [
        qdot(state.attitude, state.angular_velocity)
        parameters.J \ (u - cross(state.angular_velocity, parameters.J * state.angular_velocity))
    ]
end
