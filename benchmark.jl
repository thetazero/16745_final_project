using SatellitePlayground
using Plots
using LinearAlgebra
using ProgressMeter
SP = SatellitePlayground

include("nominal_dynamics.jl")
include("rollout.jl")
include("PlanningWithAttitude.jl")
include("tvlqr.jl")
include("trial.jl")
include("initial_conditions.jl")
PWA = PlanningWithAttitude

Random.seed!(1234)
function benchmark(orbits, states, q_err_min, q_err_max, angular_rate, plot::Bool)
  success_rate = 0
  for _ in 1:orbits
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
    @showprogress for _ in 1:states
      qic = random_quaternion_range(deg2rad(q_err_min), deg2rad(q_err_max))
      ωic = random_angular_velocity(angular_rate)
      xic = SP.RBState(
        xic.position,
        xic.velocity,
        qic,
        ωic
      )
      function log_qerr(hist, state)
        push!(hist, SP.qErr(state.attitude, xgoal.attitude))
      end
      (hist, time) = test_controller(xgoal.attitude, xic, tvlqr_control, sim_steps, sim_dt, log_step=log_qerr)
      time /= 60
      if plot
        display(plot_err(hist, xgoal.attitude, time))
      end
      if hist[end] < deg2rad(1)
        success_rate += 1
      end
    end
  end
  return success_rate / (orbits * states)
end