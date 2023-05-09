using SatellitePlayground

SP = SatellitePlayground

function tvlqr(ref_traj, As, Bs)
  N = length(ref_traj)
  Q = diagm([ones(3) * 1e2; ones(3) * 1])
  Qf = 10 * Q
  R = 1e2 * diagm(ones(3))
  nu = 3
  nx = 13

  K = [zeros(nu, nx) for i = 1:N-1]
  P = [zeros(nx, nx) for i = 1:N]
  P[N] = Qf
  for k = N-1:-1:1
    qk = ref_traj[k].attitude
    Gk = PWA.G(qk)

    A = Gk * As[k] * Gk'
    B = Gk * Bs[k]

    K[k] = (R + B' * P[k+1] * B) \ (B' * P[k+1] * A)
    P[k] = Q + K[k]' * R * K[k] + (A - B * K[k])' * P[k+1] * (A - B * K[k])
  end

  return P, K
end

function controller_gen(q0, ω0)
  function controller(x, K)
    q = x[4:7]
    ω = x[1:3]
    ϕ = PWA.qtorp(PWA.L(q0)' * q)

    Δx̃ = [ω - ω0; ϕ]
    u = -K * Δx̃
    return u
  end
end

function make_tvlqr_controller(x0::SP.RBState, J, duration, sim_dt, plan_dt)
  # reference trajectory
  plan_steps = duration / plan_dt
  (ref_traj, times) = rollout(x0, duration / plan_dt, plan_dt)
  As, Bs = generate_jacobians(ref_traj, times, J, plan_dt)

  _, K = tvlqr(ref_traj, As, Bs)
  start_time = Epoch(2020, 11, 30)

  controller = controller_gen(x0.attitude, x0.angular_velocity)

  function tvlqr_control(measurement)
    (angular_velocity, attitude, time) = measurement
    i = 1 + ((time - start_time) / plan_dt)
    i = clamp(i, 1, plan_steps)
    i = Int(floor(i))
    x = Vector([angular_velocity; attitude])
    u = controller(x, K[i])
    return Control(u)
  end

  sim_steps = duration / sim_dt
  return tvlqr_control, sim_steps
end