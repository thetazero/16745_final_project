using SatellitePlayground
using Plots
using LinearAlgebra
using ProgressLogging
SP = SatellitePlayground

include("nominal_dynamics.jl")
include("rollout.jl")
include("PlanningWithAttitude.jl")
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

# reference trajectory
time = 60 * 60 # 1 hour
dt = 0.05
steps = time / dt
@time (ref_traj, times) = rollout(x0, steps, dt)
@time As, Bs = generate_jacobians(ref_traj, times, J, dt)

# TVLQR
N = length(ref_traj)
Q = diagm([ones(3) * 1e2; ones(3) * 1])
Qf = 10 * Q
R = 1e2 * diagm(ones(3))
nu = 3
nx = 13

K = [zeros(nu, nx) for i = 1:N-1]
P = [zeros(nx, nx) for i = 1:N]
P[N] = Qf
@progress "LQR" for k = N-1:-1:1
    qk = ref_traj[k].attitude
    Gk = PWA.G(qk)

    A = Gk * As[k] * Gk'
    B = Gk * Bs[k]

    K[k] = (R + B' * P[k+1] * B) \ (B' * P[k+1] * A)
    P[k] = Q + K[k]' * R * K[k] + (A - B * K[k])' * P[k+1] * (A - B * K[k])
end

function log_state(hist, state)
    push!(hist, state)
end

function tvlqr_control(measurement)
    (state, _) = measurement
    x = Vector([state.angular_velocity; state.attitude])
    u = controller(x)
    return Control(u)
end

function controller(x)
    q0 = x0.attitude
    q = x[4:7]
    ω = x[1:3]
    ϕ = PWA.qtorp(PWA.L(q0)' * q)

    Δx̃ = [ω - x0.angular_velocity; ϕ]
    global i
    i += 1
    u = -K[i] * Δx̃
    return u
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
    i = 0
    (hist, time) = SP.simulate(tvlqr_control, log_step=log_state, max_iterations=N - 1, dt=dt,
        initial_condition=x_true, terminal_condition=terminate_on_fast_spin)
    w_hist = [norm(state.angular_velocity) for state in hist]
    q_hist = [state.attitude for state in hist]
    q_hist = SP.vec_to_mat(q_hist)
end


plot(time, w_hist, label="w")

plot(time, q_hist, label=["q1" "q2" "q3" "q4"])


# begin
#     "RK4 integrator"
#     i = 0
#     nx = 7
#     nu = 3
#     uhist = zeros(nu, N)
#     xhist = zeros(nx, N)
#     q_true = [0.9, 0.03, 0.03, 0.05]
#     q_true /= norm(q_true)
#     xhist[:, 1] = [x_true.angular_velocity; x_true.attitude]
#     for k = 1:(N-1)
#         p = attitude_params(ref_traj[k].position, J)
#         uhist[:, k] = controller(xhist[:, k])
#         @show uhist[:, k]
#         @show xhist[:, k]
#         xhist[:, k+1] = rk4(p, xhist[:, k], uhist[:, k], Epoch(2020, 1, 10) + times[k], dt, nominal_attitude_dynamics)
#     end

#     w_hist = xhist[1:3, :]
#     q_hist = xhist[4:7, :]
# end
# plot(w_hist', title="Angular Velocity", xlabel="Time (s)", ylabel="Angular Velocity (rad/s)", labels=["ω1" "ω2" "ω3" "||ω||"])
# plot(q_hist', title="Attitude", xlabel="Time (s)", ylabel="Attitude", labels=["q1" "q2" "q3" "q4"])
