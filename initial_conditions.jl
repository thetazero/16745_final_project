using SatellitePlayground
using SatelliteDynamics
using Random

SP = SatellitePlayground

# Orbital elements for reference sun synchronous orbit
sun_sync_mean_sma = SatelliteDynamics.R_EARTH + 700
sun_sync_mean_eccentricity = 0.0

function random_sun_sync(q_err_min, q_err_max, angular_rate)
  a = sun_sync_mean_sma + randn() * 30
  e = sun_sync_mean_eccentricity + randn() * 0.2
  i = SatelliteDynamics.sun_sync_inclination(a, e)
  Ω = rand() * 2 * π
  ω = randn() * 2 * π
  v = randn() * 2 * π

  orbital_elements = [a, e, i, Ω, ω, v]
  q0 = random_quaternion_range(deg2rad(q_err_min), deg2rad(q_err_max))
  ω0 = random_angular_velocity(angular_rate)
  return SatellitePlayground.state_from_osc(orbital_elements, q0, ω0)
end

function random_angular_velocity(size)
  unit = randn(3)
  unit /= norm(unit)
  return unit * size
end

# https://stackoverflow.com/questions/31600717/how-to-generate-a-random-quaternion-quickly
function random_quaternion()
  u, v, w = rand(3)
  return [sqrt(1 - u) * sin(2 * π * v), sqrt(1 - u) * cos(2 * π * v), sqrt(u) * sin(2 * π * w), sqrt(u) * cos(2 * π * w)]
end

function random_quaternion_range(low, high)
  identity = [1, 0, 0, 0]
  while true
    q = random_quaternion()
    if SP.qErr(q, identity) <= high && SP.qErr(q, identity) >= low
      return q
    end
  end
end