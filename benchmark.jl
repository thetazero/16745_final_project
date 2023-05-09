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
include("initial_conditions.jl")
PWA = PlanningWithAttitude

Random.seed!(1234)
function benchmark(N, q_err_min, q_err_max, angular_rate)
  for _ in 1:N
    J = SP.default_parameters.J
    duration = 60 * 60 # 1 hour
    sim_dt = 0.05
    plan_dt = 0.25

    xic = random_sun_sync(q_err_min, q_err_max, angular_rate)

    xgoal = SP.RBState(
      xic.position,
      xic.velocity,
      [1, 0, 0, 0],
      zeros(3)
    )

    tvlqr_control, sim_steps = make_tvlqr_controller(xgoal, J, duration, sim_dt, plan_dt)
    (hist, time) = test_controller(xgoal.attitude, xic, tvlqr_control, sim_steps, sim_dt)

    time /= 60

    display(plot_err(hist, xgoal.attitude, time))
    display(plot_attitude(hist, time))
    display(plot_angular_velocity(hist, time))
  end
end