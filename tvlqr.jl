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
  @progress "LQR" for k = N-1:-1:1
    qk = ref_traj[k].attitude
    Gk = PWA.G(qk)

    A = Gk * As[k] * Gk'
    B = Gk * Bs[k]

    K[k] = (R + B' * P[k+1] * B) \ (B' * P[k+1] * A)
    P[k] = Q + K[k]' * R * K[k] + (A - B * K[k])' * P[k+1] * (A - B * K[k])
  end

  return P, K
end

function controller(x, K)
  q0 = x0.attitude
  q = x[4:7]
  ω = x[1:3]
  ϕ = PWA.qtorp(PWA.L(q0)' * q)

  Δx̃ = [ω - x0.angular_velocity; ϕ]
  u = -K * Δx̃
  return u
end

function make_tvlqr_controller(x0::SP.RBState, p, duration)
  # reference trajectory
  dt = 0.05
  steps = duration / dt
  @time (ref_traj, times) = rollout(x0, steps, dt)
  @time As, Bs = generate_jacobians(ref_traj, times, J, dt)

  _, K = tvlqr(ref_traj, As, Bs)

  i = 0

  function tvlqr_control(measurement)
    global i
    i += 1
    (state, _) = measurement
    x = Vector([state.angular_velocity; state.attitude])
    u = controller(x, K[i])
    return Control(u)
  end
end