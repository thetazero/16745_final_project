function log_state(hist, state)
  push!(hist, state)
end

function terminate_on_fast_spin(state, parameters, time, i)
  ϕ = PWA.qtorp(PWA.L(x0.attitude)' * state.attitude)
  return norm(state.angular_velocity) > 5 || norm(ϕ) < 0.02
end

function test_controller(x_true, controller, N, dt)
  return SP.simulate(controller, log_step=log_state, max_iterations=N - 1, dt=dt,
    initial_condition=x_true, terminal_condition=terminate_on_fast_spin, measure=sim_measure)
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
  w_hist = [norm(state.angular_velocity) for state in hist]
  plot(time, w_hist, label="w")
end

