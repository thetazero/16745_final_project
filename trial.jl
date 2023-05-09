function log_state(hist, state)
  push!(hist, state)
end

function terminate_gen(q_goal)
  function terminate(state, parameters, time, i)
    return norm(state.angular_velocity) > 10 || SP.qErr(state.attitude, q_goal) < deg2rad(1)
  end
end

function test_controller(q_goal, x_true, controller, N, dt)
  terminate = terminate_gen(q_goal)
  return SP.simulate(controller, log_step=log_state, max_iterations=N - 1, dt=dt,
    initial_condition=x_true, terminal_condition=terminate, measure=sim_measure)
end

function plot_attitude(hist, time)
  q_hist = [state.attitude for state in hist]
  q_hist = SP.vec_to_mat(q_hist)
  plot(time, q_hist, label=["q1" "q2" "q3" "q4"])
end

@inline function sim_measure(state, parameters, time)
  return (state.angular_velocity, state.attitude, time)
end

function plot_angular_velocity(hist, time)
  w_hist = [rad2deg(norm(state.angular_velocity)) for state in hist]
  plot(time, w_hist, label="w", xlabel="time (m)", ylabel="angular velocity (degrees/s)")
end

function attitude_error(q1, q2) 
  return rad2deg(SP.qErr(q1, q2))
end

function plot_err(hist, ref, time)
  err_hist = [attitude_error(ref, state.attitude) for state in hist]
  plot(time, err_hist, label="attitude error", xlabel="time (m)", ylabel="error (degrees)")
end