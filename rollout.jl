using SatellitePlayground
using SatelliteDynamics
using ForwardDiff

function rollout(state, steps, dt)
    function log_step(hist, state)
        push!(hist, state)
    end
    return SatellitePlayground.simulate(control, log_step=log_step, max_iterations=steps, dt=dt,
        initial_condition=state)
end

function generate_jacobians(states, times, J)
    A = []
    B = []
    for i in eachindex(states)
        x_reff = Vector([states[i].angular_velocity; states[i].attitude])
        u_reff = [0.0, 0.0, 0.0]
        time = Epoch(2020, 11, 30) + times[i]
        push!(A,
            ForwardDiff.jacobian(
                (x) -> nominal_attitude_dynamics(x, states[i].position, J, time, u_reff),
            x_reff))
        push!(B,
            ForwardDiff.jacobian(
                (u) -> nominal_attitude_dynamics(x_reff, states[i].position, J, time, u),
                u_reff)
        )
    end
    return A, B
end