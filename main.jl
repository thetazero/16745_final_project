using SatellitePlayground
using Plots
using LinearAlgebra
using ProgressLogging
SP = SatellitePlayground

include("nominal_dynamics.jl")
include("rollout.jl")
include("PlanningWithAttitude.jl")
include("tvlqr.jl")
include("trial.jl")
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
duration = 60 * 60 # 1 hour
dt = 0.05

tvlqr_control_gen, N = make_tvlqr_controller(x0, J, duration, dt)

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

tvlqr_control = tvlqr_control_gen()
(hist, time) = test_controller(x_true, tvlqr_control, N, dt)

plot_attitude(hist, time)

plot_angular_velocity(hist, time)
