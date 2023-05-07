using SatellitePlayground
using Plots
using LinearAlgebra
using ProgressLogging
SP = SatellitePlayground

include("nominal_dynamics.jl")
include("rollout.jl")
include("PlanningWithAttitude.jl")
include("tvlqr.jl")
PWA = PlanningWithAttitude


# initial conditions
J = SP.default_parameters.J
x0 = SP.initialize_orbit()
x0 = SP.RBState(
    x0.position,
    x0.velocity,
    [1, 0, 0, 0],
    zeros(3)
)

tvlqr_control = make_tvlqr_controller(x0, J, 60 * 60)

function log_state(hist, state)
    push!(hist, state)
end

function terminate_on_fast_spin(state, parameters, time, i)
    ϕ = PWA.qtorp(PWA.L(x0.attitude)' * state.attitude)
    return norm(state.angular_velocity) > 5 || norm(ϕ) < 0.02
end

begin
    q_true = [0.99, 0.0, 0.1, 0.0]
    q_true /= norm(q_true)
    x_true = SP.RBState(
        x0.position,
        x0.velocity,
        q_true,
        [0.0, 0.0, 0.0]
    )
end

begin
    dt = 0.05
    N = 60 * 60 / dt
    (hist, time) = SP.simulate(tvlqr_control, log_step=log_state, max_iterations=N - 1, dt=dt,
        initial_condition=x_true, terminal_condition=terminate_on_fast_spin)
    w_hist = [norm(state.angular_velocity) for state in hist]
    q_hist = [state.attitude for state in hist]
    q_hist = SP.vec_to_mat(q_hist)
end


plot(time, w_hist, label="w")

plot(time, q_hist, label=["q1" "q2" "q3" "q4"])
