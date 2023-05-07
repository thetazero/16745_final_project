using SatellitePlayground
using SatelliteDynamics
using Random

# Orbital elements for reference sun synchronous orbit
sun_sync_mean_sma = SatelliteDynamics.R_EARTH + 700
sun_sync_mean_eccentricity = 0.0

function random_sun_sync()
  a = sun_sync_mean_sma + randn() * 30
  e = sun_sync_mean_eccentricity + randn() * 0.2
  i = SatelliteDynamics.sun_sync_inclination(a, e)
  Ω = randn() * 0.1
  ω = randn() * 0.1
  v = randn() * 0.1

  orbital_elements = [a, e, i, Ω, ω, v]
  q0 = [1, 0, 0, 0]
  ω0 = [0, 0, 0]
  return SatellitePlayground.state_from_osc(orbital_elements, q0, ω0)
end