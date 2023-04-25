using SatellitePlayground
using SatelliteDynamics
using ForwardDiff

function rollout(state, steps, dt)
    function log_step(hist, state)
        push!(hist, state)
    end
    function no_control(measurement)
        return zero(SatellitePlayground.Control)
    end
    return SatellitePlayground.simulate(no_control, log_step=log_step, max_iterations=steps, dt=dt,
        initial_condition=state)
end

function generate_jacobians(states, times, J, dt)
    A = []
    B = []
    for i in eachindex(states)
        x_reff = Vector([states[i].angular_velocity; states[i].attitude])
        u_reff = [0.0, 0.0, 0.0]
        time = Epoch(2020, 11, 30) + times[i]
        p = attitude_params(states[i].position, J)
        push!(A,
            ForwardDiff.jacobian(
                (x) -> rk4(p, x, u_reff, time, dt, nominal_attitude_dynamics),
                x_reff))
        push!(B,
            ForwardDiff.jacobian(
                (u) -> rk4(p, x_reff, u, time, dt, nominal_attitude_dynamics),
                u_reff))
    end
    return A, B
end