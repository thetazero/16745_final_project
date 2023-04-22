using SatellitePlayground
using ForwardDiff

function rollout(state, steps, dt)
    function log_step(hist, state)
        push!(hist, state)
    end
    function log_end(hist)
        return hist
    end
    function terminate(state, params, time, i)
        return false
    end
    return SatellitePlayground.simulate(control, log_step=log_step, max_iterations=steps, dt=dt,
        terminal_condition=terminate, log_end=log_end, initial_condition=state)
end

function generate_jacobians(state, steps, dt, J)
    (states, times) = rollout(state, steps, dt)
    A = []
    B = []
    for i in eachindex(states)
        x_reff = Vector([states[i].angular_velocity; states[i].attitude])
        u_reff = [0.0, 0.0, 0.0]
        time = times[i]
        push!(A,
            ForwardDiff.jacobian(
                (x) -> nominal_attitude_dynamics(x, state, J, time, u_reff)),
            x_reff)
        push!(B,
            ForwardDiff.jacobian(
                (u) -> nominal_attitude_dynamics(x_reff, state, J, time, u),
                u_reff)
        )
    end
end